# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# AFC AutoCal — per-spool flow calibration (K), all-or-nothing.
#
# Decoupled from any RFID reader: it keys purely off a lane's spool_id, so it
# works no matter how the spool was identified (RFID, scanner, or manual
# SET_SPOOL_ID).
#
# Two independent toggles, applied to ANY lane on every tool load:
#   apply_stored_k -> if the spool has a stored K in Spoolman, apply it (and
#                     re-apply after homing / extruder activation).
#   auto_calibrate -> if the spool has NO stored K, run a calibration and store
#                     the result on the Spoolman spool.
# Use apply_stored_k alone to apply saved K without ever auto-calibrating.
# With both off the module does nothing.
#
# K is persisted per-spool in Spoolman's comment field (afc_flow_k=<value>),
# read/written via the shared SpoolmanClient. The actual measurement is the
# ``calibrate_gcode`` command (default FLOW_CALIBRATE) — point it at a macro
# that handles any purge/discard-bin mechanics if needed.
#
# ── Configuration ───────────────────────────────────────────────────
#   [AFC_autocal]
#   apply_stored_k: True              # apply a spool's saved K on load
#   auto_calibrate: False             # calibrate + store when no saved K exists
#   # enabled: True                   # back-compat master: turns BOTH on
#   calibrate_gcode: FLOW_CALIBRATE   # command run to measure K (default)

from __future__ import annotations
import logging
import threading

from extras.AFC_RFID import SpoolmanClient


class AFC_autocal:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.logger = logging.getLogger('AFC_autocal')
        self.afc = None
        self._lane_flow_k = {}   # lane_name -> (spool_id, k)
        self._k_fetch_inflight = set()   # lane names with a pending async read

        # Two independent toggles. 'enabled' is a back-compat master that
        # defaults BOTH on when set.
        #   apply_stored_k - apply a spool's saved K on load + re-apply on homing
        #   auto_calibrate - if a loaded spool has NO saved K, calibrate & store
        master = config.getboolean('enabled', None)
        dflt = master if master is not None else False
        self.apply_stored_k = config.getboolean('apply_stored_k', dflt)
        self.auto_calibrate = config.getboolean('auto_calibrate', dflt)
        self.calibrate_gcode = config.get('calibrate_gcode', 'FLOW_CALIBRATE')

        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.printer.register_event_handler('afc:tool_loaded',
                                            self._handle_tool_loaded)
        self.printer.register_event_handler('homing:home_rails_end',
                                            self._handle_home_rails_end)
        self.printer.register_event_handler('extruder:activate_extruder',
                                            self._handle_activate_extruder)

        self.gcode.register_command(
            'AFC_APPLY_LANE_FLOW_K', self.cmd_APPLY_LANE_FLOW_K,
            desc="Apply stored flow K for the current lane")
        self.gcode.register_command(
            'AFC_CALIBRATE_LANE_FLOW_K', self.cmd_CALIBRATE_LANE_FLOW_K,
            desc="Run flow calibration on the current lane and store K")

    # ── Lifecycle ───────────────────────────────────────────────────

    def _handle_ready(self):
        self.afc = self.printer.lookup_object('AFC', None)
        if self.afc is None:
            self.logger.warning("AFC_autocal: AFC not loaded; disabled")
            return
        self.logger = self.afc.logger
        if self.apply_stored_k or self.auto_calibrate:
            self._patch_set_tool_loaded_emit()

    def _patch_set_tool_loaded_emit(self):
        """Make every normal toolchange emit ``afc:tool_loaded``.

        Upstream ``AFCLane.set_tool_loaded()`` doesn't fire the event; only the
        ACE/OpenAMS/U1 units emit it in their own special paths, so a plain lane
        load (e.g. a standalone U1 extruder, or a Box Turtle / HTLF lane) never
        triggers autocal. Patch the lane base class here — entirely inside
        AFC_autocal — so this behavior exists ONLY when this module is
        installed/enabled and we never touch the frozen upstream files. The
        patch is idempotent (guarded by a class flag) and a no-op for the
        startup-reconcile / RFID emit paths, which set lane status directly
        rather than going through set_tool_loaded.
        """
        try:
            from extras.AFC_lane import AFCLane
        except Exception as e:
            self.logger.warning(
                "AFC_autocal: cannot patch set_tool_loaded: %s" % e)
            return
        if getattr(AFCLane, '_afc_autocal_emit_patched', False):
            return
        _orig = AFCLane.set_tool_loaded

        def set_tool_loaded(self, normal_toolchange=False, _orig=_orig):
            _orig(self, normal_toolchange=normal_toolchange)
            # Emit on every load, not just normal_toolchange: the direct
            # "load to extruder" path (AFC_extruder.temp_check_cb) and the
            # OpenAMS load paths call set_tool_loaded() with the default
            # (normal_toolchange=False), and those loads need autocal too.
            try:
                self.printer.send_event("afc:tool_loaded", self)
            except Exception:
                pass

        AFCLane.set_tool_loaded = set_tool_loaded
        AFCLane._afc_autocal_emit_patched = True
        self.logger.info(
            "AFC_autocal: set_tool_loaded now emits afc:tool_loaded on load")

    def _spoolman(self):
        mr = getattr(self.afc, 'moonraker', None) if self.afc else None
        if mr is None or getattr(self.afc, 'spoolman', None) is None:
            return None
        return SpoolmanClient(mr)

    # ── K cache (spool-validated) ───────────────────────────────────

    @staticmethod
    def _norm_spool_id(sid):
        if sid in (None, "", 0, "0"):
            return None
        try:
            return int(sid)
        except (ValueError, TypeError):
            return None

    def _set_lane_k(self, lane, k):
        self._lane_flow_k[lane.name] = (
            self._norm_spool_id(getattr(lane, 'spool_id', None)), k)

    def _get_lane_k(self, lane):
        entry = self._lane_flow_k.get(lane.name)
        if entry is None:
            return None
        stored, k = entry
        if stored != self._norm_spool_id(getattr(lane, 'spool_id', None)):
            del self._lane_flow_k[lane.name]
            return None
        return k

    # ── Spoolman read/write (via the shared client) ─────────────────

    def _read_k_from_spoolman(self, lane):
        sid = self._norm_spool_id(getattr(lane, 'spool_id', None))
        if sid is None:
            return None
        client = self._spoolman()
        return client.read_flow_k(sid) if client else None

    def _write_k_to_spoolman(self, lane, k):
        sid = self._norm_spool_id(getattr(lane, 'spool_id', None))
        if sid is None:
            return
        client = self._spoolman()
        if client is not None:
            client.write_flow_k(sid, k)

    # ── Apply via flow_calibrator ───────────────────────────────────

    def _apply_lane_k(self, lane_name):
        entry = self._lane_flow_k.get(lane_name)
        if entry is None:
            return None
        _, k = entry
        flow_cal = self.printer.lookup_object('flow_calibrator', None)
        if flow_cal is None:
            self.logger.warning("AFC_autocal: flow_calibrator not found")
            return None
        ext = self.printer.lookup_object('toolhead').get_extruder()
        ext_name = ext.get_name()
        flow_cal._set_pressure_advance(ext, k)
        flow_cal._current_k[ext_name] = k
        msg = ("AFC autocal: applied K=%.6f for %s on %s"
               % (k, lane_name, ext_name))
        self.logger.info(msg)
        return msg

    def _ensure_k_loaded(self, lane):
        """Return K for a lane: cached, else from Spoolman. None if neither."""
        k = self._get_lane_k(lane)
        if k is None:
            k = self._read_k_from_spoolman(lane)
            if k is not None:
                self._set_lane_k(lane, k)
        return k

    def _calibrate(self, cur_lane, gcmd=None, runner=None):
        """Run the calibration command, then store/apply/persist the new K.

        :param runner: callable used to run ``calibrate_gcode``. Defaults to
          ``run_script_from_command`` (assumes the gcode mutex is held, i.e.
          we're inside a command). Pass ``self.gcode.run_script`` when calling
          from a deferred/reactor context so the mutex is acquired safely.
        :return float or None: the measured K, or None if none produced.
        """
        flow_cal = self.printer.lookup_object('flow_calibrator', None)
        if flow_cal is None:
            msg = "AFC_autocal: flow_calibrator not found"
            if gcmd is not None:
                gcmd.respond_info(msg)
            else:
                self.logger.warning(msg)
            return None
        ext_name = cur_lane.extruder_obj.name
        k_before = flow_cal._current_k.get(ext_name)
        run = runner if runner is not None else self.gcode.run_script_from_command
        run(self.calibrate_gcode)
        k_after = flow_cal._current_k.get(ext_name)
        if k_after is None or k_after == k_before:
            self.logger.info(
                "AFC autocal: calibration produced no new K for %s"
                % cur_lane.name)
            return None
        self._set_lane_k(cur_lane, k_after)
        self._apply_lane_k(cur_lane.name)
        self._write_k_to_spoolman(cur_lane, k_after)
        self.logger.info(
            "AFC autocal: calibrated and stored K=%.6f for %s"
            % (k_after, cur_lane.name))
        return k_after

    def _current_lane(self):
        if self.afc is None:
            return None
        try:
            return self.afc.function.get_current_lane_obj()
        except Exception:
            return None

    # ── Event handlers ──────────────────────────────────────────────

    def _handle_tool_loaded(self, cur_lane):
        if self.afc is None or cur_lane is None:
            return
        if not (self.apply_stored_k or self.auto_calibrate):
            return
        # set_tool_loaded fires mid-toolchange with the gcode mutex held, so we
        # can't apply K / run a calibration synchronously here. Defer to the
        # reactor; _do_tool_loaded then uses run_script (which acquires the
        # mutex) and runs once the load has released it.
        self.reactor.register_callback(
            lambda et, lane=cur_lane: self._do_tool_loaded(lane))

    def _do_tool_loaded(self, cur_lane):
        try:
            if self.afc is None or cur_lane is None:
                return
            if not (self.apply_stored_k or self.auto_calibrate):
                return
            # An already-CACHED K is just an MCU command (no I/O) — apply it
            # immediately, safe any time including mid-print tool changes.
            if self._get_lane_k(cur_lane) is not None:
                if self.apply_stored_k:
                    self._apply_lane_k(cur_lane.name)
                return
            # Uncached: reading K from Spoolman is a SYNCHRONOUS HTTP call.
            # Run it OFF the reactor thread so it can't stall step delivery and
            # trip the MCU "Timer too close"; the result is applied back on the
            # reactor by _k_applied.
            self._fetch_k_async(cur_lane)
        except Exception as e:
            self.logger.warning("AFC_autocal: tool_loaded error: %s" % e)

    def _fetch_k_async(self, cur_lane):
        """Read this lane's K from Spoolman in a worker thread, then hand the
        result to the reactor. The worker only does the (stateless) HTTP read;
        all state changes / applies happen on the reactor in _k_applied."""
        lane_name = cur_lane.name
        if lane_name in self._k_fetch_inflight:
            return
        sid = self._norm_spool_id(getattr(cur_lane, 'spool_id', None))
        if sid is None:
            # No spool to look up — nothing to apply; calibrate if idle/enabled.
            if self.auto_calibrate and self._safe_to_calibrate():
                self._calibrate(cur_lane, runner=self.gcode.run_script)
            return
        if not self.apply_stored_k:
            # Only auto_calibrate is on, and that needs idle — defer to _k_applied
            # with k=None so it takes the (idle-gated) calibrate path.
            self._k_applied(lane_name, sid, None)
            return

        self._k_fetch_inflight.add(lane_name)

        def worker():
            k = None
            try:
                client = self._spoolman()
                if client is not None:
                    k = client.read_flow_k(sid)
            except Exception as e:
                self.logger.debug("AFC_autocal: async K read failed: %s" % e)
            # Hop back onto the reactor (thread-safe) to apply / decide.
            self.reactor.register_async_callback(
                lambda et: self._k_applied(lane_name, sid, k))

        threading.Thread(target=worker, name="afc-autocal-k",
                         daemon=True).start()

    def _k_applied(self, lane_name, sid, k):
        """Runs on the reactor with the K read off-thread. Re-validate the lane
        still carries that spool (it may have changed during the read), then
        apply the stored K or fall back to an idle-gated calibration."""
        self._k_fetch_inflight.discard(lane_name)
        try:
            lane = self.afc.lanes.get(lane_name) if self.afc else None
            if lane is None:
                return
            if self._norm_spool_id(getattr(lane, 'spool_id', None)) != sid:
                return  # spool changed since we kicked off the read — stale
            if k is not None:
                self._set_lane_k(lane, k)
                if self.apply_stored_k:
                    self._apply_lane_k(lane_name)
                return
            # No stored K — optionally calibrate (stores + applies). Gated to
            # prep-done + idle so we never start a calibration mid-print.
            if self.auto_calibrate and self._safe_to_calibrate():
                self._calibrate(lane, runner=self.gcode.run_script)
        except Exception as e:
            self.logger.warning("AFC_autocal: K apply error: %s" % e)

    def _safe_to_calibrate(self):
        if not getattr(self.afc, 'prep_done', False):
            return False
        state = getattr(self.afc, 'current_state', None)
        if state is not None and str(state).split('.')[-1].lower() != 'idle':
            return False
        return True

    def _reapply_current_k(self):
        # Re-applying only matters when we apply stored K.
        if not self.apply_stored_k or self.afc is None:
            return
        if not getattr(self.afc, 'prep_done', False):
            return
        state = getattr(self.afc, 'current_state', None)
        if state is not None and str(state).split('.')[-1].lower() != 'idle':
            return
        cur_lane = self._current_lane()
        if cur_lane is None:
            return
        # Re-apply only the already-cached K (no Spoolman read on this event
        # path). The K was cached at load; if it wasn't, the load's async fetch
        # applies it — no need to block here.
        if self._get_lane_k(cur_lane) is not None:
            self._apply_lane_k(cur_lane.name)

    def _handle_home_rails_end(self, homing_state, rails):
        try:
            self._reapply_current_k()
        except Exception as e:
            self.logger.warning("AFC_autocal: home reapply error: %s" % e)

    def _handle_activate_extruder(self):
        try:
            self._reapply_current_k()
        except Exception as e:
            self.logger.warning("AFC_autocal: activate reapply error: %s" % e)

    # ── GCode commands (manual overrides, work regardless of enabled) ─

    def cmd_APPLY_LANE_FLOW_K(self, gcmd):
        cur_lane = self._current_lane()
        if cur_lane is None:
            gcmd.respond_info("AFC_autocal: no current lane")
            return
        if self._ensure_k_loaded(cur_lane) is None:
            gcmd.respond_info("AFC_autocal: no stored K for %s" % cur_lane.name)
            return
        gcmd.respond_info(self._apply_lane_k(cur_lane.name) or "applied")

    def cmd_CALIBRATE_LANE_FLOW_K(self, gcmd):
        cur_lane = self._current_lane()
        if cur_lane is None:
            gcmd.respond_info("AFC_autocal: no current lane")
            return
        k = self._calibrate(cur_lane, gcmd=gcmd)
        if k is not None:
            gcmd.respond_info(
                "AFC_autocal: stored K=%.6f for %s" % (k, cur_lane.name))
        else:
            gcmd.respond_info("AFC_autocal: calibration produced no new K")


def load_config(config):
    return AFC_autocal(config)
