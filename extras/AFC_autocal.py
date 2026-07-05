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
        self._staged_handled = {}   # lane_name -> spool_id auto-loaded from stage
        self._staged_pending = set()   # lanes with a retry chain awaiting idle
        self._cal_pending = set()   # lanes with a calibrate-when-idle chain

        # Two independent toggles. 'enabled' is a back-compat master that
        # defaults BOTH on when set.
        #   apply_stored_k - apply a spool's saved K on load + re-apply on homing
        #   auto_calibrate - if a loaded spool has NO saved K, calibrate & store
        master = config.getboolean('enabled', None)
        dflt = master if master is not None else False
        self.apply_stored_k = config.getboolean('apply_stored_k', dflt)
        self.auto_calibrate = config.getboolean('auto_calibrate', dflt)
        self.calibrate_gcode = config.get('calibrate_gcode', 'FLOW_CALIBRATE')
        # Suppress auto-calibration for this many seconds after klippy:ready so
        # the startup prep reconcile (which fires once prep_done flips) doesn't
        # kick off a calibration; genuine spool inserts later still calibrate.
        self._startup_cal_grace = config.getfloat('startup_cal_grace', 30.0)
        self._ready_time = None

        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.printer.register_event_handler('afc:tool_loaded',
                                            self._handle_tool_loaded)
        self.printer.register_event_handler('afc:spool_assigned',
                                            self._handle_spool_assigned)
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
        self._ready_time = self.reactor.monotonic()
        self.afc = self.printer.lookup_object('AFC', None)
        if self.afc is None:
            self.logger.warning("AFC_autocal: AFC not loaded; disabled")
            return
        self.logger = self.afc.logger
        if self.apply_stored_k or self.auto_calibrate:
            self._patch_set_tool_loaded_emit()
        if self.auto_calibrate:
            self._patch_set_spoolid_emit()

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
    # TODO: remove patch, need to stop patching things claude
    def _patch_set_spoolid_emit(self):
        """Make spool assignment emit ``afc:spool_assigned``.

        A spool staged into a lane (any unit type) is assigned via
        ``AFCSpool.set_spoolID`` but isn't loaded to the toolhead, so it never
        fires ``afc:tool_loaded`` and autocal has nothing to hook. Patch
        set_spoolID here (entirely inside AFC_autocal, idempotent) to emit the
        lane on every call; _handle_spool_assigned then auto-loads an uncalibrated
        staged lane so the normal tool_loaded path calibrates it. Heavy gating in
        the handler (filament present, idle, prep-done, startup grace, dedup)
        keeps startup reconcile / clears from triggering anything.
        """
        try:
            from extras.AFC_spool import AFCSpool
        except Exception as e:
            self.logger.warning(
                "AFC_autocal: cannot patch set_spoolID: %s" % e)
            return
        if getattr(AFCSpool, '_afc_autocal_spoolid_patched', False):
            return
        _orig = AFCSpool.set_spoolID

        def set_spoolID(self, cur_lane, SpoolID, save_vars=True, _orig=_orig):
            _orig(self, cur_lane, SpoolID, save_vars=save_vars)
            try:
                self.printer.send_event("afc:spool_assigned", cur_lane)
            except Exception:
                pass

        AFCSpool.set_spoolID = set_spoolID
        AFCSpool._afc_autocal_spoolid_patched = True
        self.logger.info(
            "AFC_autocal: set_spoolID now emits afc:spool_assigned")

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
        # flow_calibrator keys _current_k by the Klipper toolhead extruder name,
        # which differs from the AFC_extruder section name when the extruder is
        # renamed (v1.1.22 'extruder_name'). Use the Klipper name.
        ext_obj = cur_lane.extruder_obj
        ext_name = getattr(ext_obj, 'th_extruder_name', None) or ext_obj.name
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
            # immediately, but only when this lane's tool is the active one
            # (applying targets the mounted extruder). For an off-shuttle lane
            # it's re-applied when its tool is activated.
            if self._get_lane_k(cur_lane) is not None:
                if self.apply_stored_k and self._lane_on_active_toolhead(cur_lane):
                    self._apply_lane_k(cur_lane.name)
                return
            # Uncached: reading K from Spoolman is a SYNCHRONOUS HTTP call.
            # Run it OFF the reactor thread so it can't stall step delivery and
            # trip the MCU "Timer too close"; the result is applied back on the
            # reactor by _k_applied.
            self._fetch_k_async(cur_lane)
        except Exception as e:
            self.logger.warning("AFC_autocal: tool_loaded error: %s" % e)

    def _handle_spool_assigned(self, cur_lane):
        if self.afc is None or cur_lane is None or not self.auto_calibrate:
            return
        # set_spoolID may run with the gcode mutex held — defer to the reactor.
        self.reactor.register_callback(
            lambda et, lane=cur_lane: self._do_spool_assigned(lane))

    def _do_spool_assigned(self, cur_lane, attempts=0):
        """A spool was assigned to a lane. If the lane is staged (filament
        present, has a spool, but isn't in the toolhead) and uncalibrated,
        auto-load it once it's safe so the normal tool_loaded path calibrates it.
        Works with any unit type. A lane already in the toolhead is handled by the
        tool_loaded path; a direct extruder auto-loading is skipped because it
        becomes tool_loaded (or is still load-in-flight) while we wait. Spools
        already staged at boot are left alone (startup grace)."""
        try:
            if self.afc is None or cur_lane is None or not self.auto_calibrate:
                return
            name = cur_lane.name
            sid = self._norm_spool_id(getattr(cur_lane, 'spool_id', None))
            if sid is None:
                # Spool cleared/removed — allow a later re-insert to retry.
                self._staged_handled.pop(name, None)
                self._staged_pending.discard(name)
                return
            if getattr(cur_lane, 'tool_loaded', False):
                self._staged_pending.discard(name)
                return  # in the toolhead — tool_loaded path handles it
            if not getattr(cur_lane, 'load_state', False):
                self._staged_pending.discard(name)
                return  # no filament present at the lane — nothing to load
            if self._staged_handled.get(name) == sid:
                return  # already auto-loaded this spool from staging
            if attempts == 0 and name in self._staged_pending:
                return  # a retry chain is already waiting for this lane
            # Don't auto-load spools already staged at boot: inside the startup
            # grace we neither act nor retry (genuine inserts happen later).
            now = self.reactor.monotonic()
            if (self._ready_time is not None
                    and (now - self._ready_time) < self._startup_cal_grace):
                self._staged_pending.discard(name)
                return
            # The assignment often fires mid staging-load, so wait (bounded) for
            # the printer to be idle/prepped AND for any extruder load in flight
            # to finish (a direct extruder auto-loading becomes tool_loaded —
            # let it, rather than forcing a redundant tool change).
            if (not self._safe_to_calibrate()
                    or self._extruder_load_in_flight(cur_lane)):
                if attempts < 30:
                    self._staged_pending.add(name)
                    self.reactor.register_callback(
                        lambda et, l=cur_lane, a=attempts + 1:
                            self._do_spool_assigned(l, a), now + 1.0)
                else:
                    self._staged_pending.discard(name)
                return
            self._staged_pending.discard(name)
            self._staged_handled[name] = sid
            # Only auto-load when it actually needs calibration (no stored K).
            # The K read is a blocking HTTP call, so do it off the reactor.
            self._check_staged_k_async(name, sid)
        except Exception as e:
            self.logger.warning("AFC_autocal: spool_assigned error: %s" % e)

    def _check_staged_k_async(self, lane_name, sid):
        def worker():
            k = None
            read_ok = False
            try:
                client = self._spoolman()
                if client is not None:
                    k = client.read_flow_k(sid)
                    read_ok = True
            except Exception as e:
                self.logger.debug("AFC_autocal: staged K read failed: %s" % e)
            self.reactor.register_async_callback(
                lambda et: self._staged_k_ready(lane_name, sid, k, read_ok))

        threading.Thread(target=worker, name="afc-autocal-staged",
                         daemon=True).start()

    def _staged_k_ready(self, lane_name, sid, k, read_ok):
        """Runs on the reactor with the staged spool's K. Auto-load (to calibrate)
        ONLY when we positively confirmed the spool has no stored K: a spool that
        already has a K, or one whose K we couldn't read, is left staged."""
        try:
            lane = self.afc.lanes.get(lane_name) if self.afc else None
            if lane is None:
                return
            if self._norm_spool_id(getattr(lane, 'spool_id', None)) != sid:
                return  # spool changed since we kicked off the read
            if getattr(lane, 'tool_loaded', False):
                return  # got loaded in the meantime
            if k is not None:
                self._set_lane_k(lane, k)
                self.logger.info(
                    "AFC autocal: %s spool %s already has K=%.6f — not auto-loading"
                    % (lane_name, sid, k))
                return  # already calibrated — don't force a load
            if not read_ok:
                self.logger.info(
                    "AFC autocal: %s spool %s K unknown (Spoolman read failed) "
                    "— not auto-loading" % (lane_name, sid))
                return  # couldn't confirm there's no K — be conservative
            if not self._safe_to_calibrate():
                return
            self.logger.info(
                "AFC autocal: %s spool %s has no stored K — loading to calibrate"
                % (lane_name, sid))
            self.gcode.run_script("CHANGE_TOOL LANE=%s" % lane_name)
        except Exception as e:
            self.logger.warning("AFC_autocal: staged load error: %s" % e)

    def _fetch_k_async(self, cur_lane):
        """Read this lane's K from Spoolman in a worker thread, then hand the
        result to the reactor. The worker only does the (stateless) HTTP read;
        all state changes / applies happen on the reactor in _k_applied."""
        lane_name = cur_lane.name
        if lane_name in self._k_fetch_inflight:
            return
        sid = self._norm_spool_id(getattr(cur_lane, 'spool_id', None))
        if sid is None:
            # No spool to look up — nothing to apply; calibrate if enabled.
            # _calibrate_when_loaded waits for the printer to settle to idle and
            # skips during a print. Not gated to the active tool: calibrate_gcode
            # switches to this lane's tool and loads it before measuring.
            if self.auto_calibrate:
                self._calibrate_when_loaded(cur_lane)
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
                # Cache it regardless, but only apply when this lane's tool is
                # mounted (else it'd set the active tool's pressure advance).
                if self.apply_stored_k and self._lane_on_active_toolhead(lane):
                    self._apply_lane_k(lane_name)
                return
            # No stored K — optionally calibrate (stores + applies). Gated to
            # prep-done + idle (+ startup grace) so we never start a calibration
            # mid-print or during boot prep. NOT gated to the active tool:
            # calibrate_gcode switches to this lane's tool and loads it before
            # measuring, so an inserted off-shuttle lane calibrates too.
            if self.auto_calibrate:
                self._calibrate_when_loaded(lane)
        except Exception as e:
            self.logger.warning("AFC_autocal: K apply error: %s" % e)

    def _extruder_load_in_flight(self, lane):
        """True while the lane's extruder is mid async-load. The U1 direct-load
        (AFC_extruder.move_extruder) schedules a deferred cleanup timer
        (extruder_move_cb) that calls toolhead.flush_step_generation(); if a
        calibration's homing/drip move overlaps that cleanup, the flush raises
        DripModeEndSignal and shuts Klipper down. Non-U1 loads don't set this
        flag, so they calibrate immediately as before."""
        ext = getattr(lane, 'extruder_obj', None)
        return bool(getattr(ext, 'load_active', False))

    def _is_printing(self):
        try:
            return bool(self.afc.function.is_printing())
        except Exception:
            return False

    def _calibrate_when_loaded(self, lane, attempts=0):
        """Run the flow calibration once the printer settles to idle after the
        load. The auto-load issues a full CHANGE_TOOL, so the AFC state stays
        non-idle ('Loading') for the whole sequence (~90s) — wait (bounded) for
        it to finish rather than skipping on the first non-idle read. Never
        calibrate during a print, before prep, or in the startup grace, and wait
        out any in-flight extruder load so its cleanup can't overlap the
        calibration's homing/drip move (see _extruder_load_in_flight)."""
        name = lane.name
        if attempts == 0:
            if name in self._cal_pending:
                return  # a calibrate chain is already running for this lane
            self._cal_pending.add(name)
        try:
            if self._is_printing():
                self.logger.info(
                    "AFC autocal: %s calibration skipped — printing" % name)
                self._cal_pending.discard(name)
                return
            reason = self._cal_block_reason()
            # Hard stops we don't wait out: boot prep / startup grace.
            if reason == "prep not done" or (
                    reason is not None and reason.startswith("within startup")):
                self.logger.info(
                    "AFC autocal: %s calibration skipped — %s" % (name, reason))
                self._cal_pending.discard(name)
                return
            # Transient: the tool change / load hasn't settled to idle yet, or an
            # extruder load is still in flight. Wait for it (bounded ~4min).
            if reason is not None or self._extruder_load_in_flight(lane):
                if attempts < 240:
                    self.reactor.register_callback(
                        lambda et, l=lane, a=attempts + 1:
                            self._calibrate_when_loaded(l, a),
                        self.reactor.monotonic() + 1.0)
                else:
                    self.logger.info(
                        "AFC autocal: %s calibration gave up waiting to settle "
                        "(%s)" % (name, reason or "load in flight"))
                    self._cal_pending.discard(name)
                return
            self._cal_pending.discard(name)
            if not getattr(lane, 'tool_loaded', False):
                return  # lane was unloaded while we waited
            # FLOW_CALIBRATE only measures the ACTIVE toolhead extruder — it does
            # not pick up a tool. So only calibrate when this lane's tool is the
            # one mounted (a genuine insert is auto-loaded onto the toolhead
            # first, so it passes; a lane merely marked loaded via SET_LANE_LOADED
            # on an off-shuttle tool would otherwise calibrate the wrong extruder).
            if not self._lane_on_active_toolhead(lane):
                try:
                    active = self.printer.lookup_object(
                        'toolhead').get_extruder().get_name()
                except Exception:
                    active = '?'
                self.logger.info(
                    "AFC autocal: %s calibration skipped — its tool is not on "
                    "the toolhead (active extruder=%s); pick up/load this lane's "
                    "tool to calibrate it" % (name, active))
                return
            self.logger.info(
                "AFC autocal: running flow calibration for %s" % name)
            self._calibrate(lane, runner=self.gcode.run_script)
        except Exception as e:
            self._cal_pending.discard(name)
            self.logger.warning("AFC_autocal: deferred calibrate error: %s" % e)

    def _lane_on_active_toolhead(self, lane):
        """True when this lane's extruder is the one currently mounted on the
        toolhead. Applying stored K sets pressure advance on the ACTIVE extruder,
        so we only apply a lane's K when its tool is actually mounted, never on a
        sibling tool on the shuttle (e.g. T0 during prep). An off-shuttle lane's
        cached K is re-applied when its tool is activated. (Calibration is NOT
        gated this way — calibrate_gcode switches/loads to the lane's tool.)"""
        try:
            ext_obj = getattr(lane, 'extruder_obj', None)
            if ext_obj is None:
                return False
            active = self.printer.lookup_object('toolhead').get_extruder()
            if active is None:
                return False
            # The toolhead reports the Klipper extruder name; an AFC_extruder
            # section can be renamed (v1.1.22 'extruder_name'), so match either
            # the section name or the Klipper name (th_extruder_name).
            return active.get_name() in (
                getattr(ext_obj, 'name', None),
                getattr(ext_obj, 'th_extruder_name', None))
        except Exception:
            return False

    def _safe_to_calibrate(self):
        return self._cal_block_reason() is None

    def _cal_block_reason(self):
        """None when it's safe to calibrate, else a short reason string."""
        if not getattr(self.afc, 'prep_done', False):
            return "prep not done"
        if (self._ready_time is not None
                and (self.reactor.monotonic() - self._ready_time)
                < self._startup_cal_grace):
            return "within startup grace"
        state = getattr(self.afc, 'current_state', None)
        if state is not None and str(state).split('.')[-1].lower() != 'idle':
            return "state=%s (not idle)" % str(state).split('.')[-1]
        return None

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
