# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# U1 RFID integration: reads spool data from Snapmaker U1's filament_detect
# Klipper module and syncs to AFC lanes / Spoolman.

from __future__ import annotations
import logging
import re
from typing import TYPE_CHECKING, Optional, Dict

if TYPE_CHECKING:
    from extras.AFC import afc
    from extras.AFC_lane import AFCLane

from extras.AFC_RFID import (
    color_name, color_label, color_distance, density_for_material,
    log_new_filament, log_new_spool,
    get_auto_spoolman_create, apply_filament_defaults,
    sync_rfid_to_spoolman,
)

POLL_INTERVAL = 2.0
_MAX_CONSECUTIVE_FAILURES = 5
_BACKOFF_INTERVAL = 10.0
_BACKOFF_RESET_CYCLES = 18  # ~3 min at 10s intervals before retrying normal speed
_FORCE_READ_TIMEOUT = 1.0
_FORCE_READ_POLL_STEP = 0.05


class AFC_U1_RFID:
    """Polls the Snapmaker U1 filament_detect Klipper object for RFID tag data
    and applies it to AFC lanes (material, color) and Spoolman."""

    def __init__(self, config):
        """Configure the reader from its ``[AFC_U1_rfid]`` section.

        Parses lane->channel maps, standalone scanner channels, auto-create
        options and the OpenRFID webhook grace, registers the
        ``afc/u1_rfid`` webhook endpoint, and defers wiring to ``klippy:ready``.

        :param config: Klipper ConfigWrapper for the ``[AFC_U1_rfid]`` section.
        """
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.logger = logging.getLogger('AFC_U1_rfid')
        self.afc = None
        self._filament_detect = None
        self._lane_channel_map: Dict[str, int] = {}
        self._lane_objects: Dict[str, AFCLane] = {}
        self._last_uid: Dict[int, Optional[list]] = {}
        self._poll_timer = None
        self._scanner_channels: set = set()
        self._channel_to_lane: Dict[int, str] = {}
        self._consecutive_failures: Dict[int, int] = {}
        self._backed_off: bool = False
        self._backoff_cycles: int = 0
        self._fd_cb_registered: bool = False
        # The reader owns its own config and lifecycle: the lane->channel map
        # and scanner channels are configured in THIS section, and the reader
        # wires up its own polling — fully decoupled from the AFC core.
        #   [AFC_U1_rfid]
        #   lane_channels: lane4:1, lane5:2, lane6:3   # tag -> assign to lane
        #   scanner_channels: 0                         # tag -> stage next spool
        #   scanner_auto_create: True   # opt-in (default False): create scanned
        #                               # spools in Spoolman when no match exists
        # lane_channels: a loadable AFC lane reads its RFID channel and the tag
        # is assigned to THAT lane. (Alias: 'channels'.)
        self._cfg_channels: Dict[str, int] = {}        # lane_name -> channel
        lane_chan_str = config.get('lane_channels', None)
        if lane_chan_str is None:
            lane_chan_str = config.get('channels', '')
        for pair in lane_chan_str.split(','):
            pair = pair.strip()
            if not pair:
                continue
            name, sep, ch = pair.partition(':')
            if not sep:
                raise config.error(
                    "AFC_U1_rfid: 'lane_channels' entries must be "
                    "'lane:channel', got '%s'" % pair)
            try:
                self._cfg_channels[name.strip()] = int(ch.strip())
            except ValueError:
                raise config.error(
                    "AFC_U1_rfid: bad channel number in '%s'" % pair)
        # scanner_channels: standalone spool-scanner RFID channels with NO lane.
        # Scanning one stages the spool as next_spool_id for whatever lane loads
        # next — so a scanner is never tied to (and can't collide with) a lane
        # name like an OpenAMS 'lane0'.
        self._cfg_scanner_channels: set = set()
        for ch in config.get('scanner_channels', '').split(','):
            ch = ch.strip()
            if not ch:
                continue
            try:
                self._cfg_scanner_channels.add(int(ch))
            except ValueError:
                raise config.error(
                    "AFC_U1_rfid: bad scanner channel '%s'" % ch)
        # Auto-create scanned spools in Spoolman (scanner channels have no lane,
        # so the lane-based auto-create lookup doesn't apply). Opt-in: defaults
        # off so creating Spoolman entries is an explicit choice.
        self._scanner_auto_create = config.getboolean(
            'scanner_auto_create', False)
        # Default auto-create for LANE reads via this reader (extruder1/2/3 etc.
        # whose unit/extruder are upstream-frozen and can't take the option). A
        # lane's unit/extruder auto_spoolman_create still overrides this.
        self._lane_auto_create = config.getboolean(
            'auto_spoolman_create', False)
        # scanner_lanes: a loadable lane that ALSO acts as a scanner (rare).
        self._cfg_scanners: set = {s.strip() for s in
                                   config.get('scanner_lanes', '').split(',')
                                   if s.strip()}
        # OpenRFID full-colour webhook. The filament_detect dict only carries
        # the primary colour (the daemon's success_exporter serialises just
        # RGB_1), so dual/gradient spools lose their real second colour. If the
        # OpenRFID config POSTs the full GenericFilament to /printer/afc/u1_rfid
        # (a [webhook_exporter] block), we use that complete colour list. No
        # separate enable flag — configuring [AFC_U1_rfid] is the opt-in, and we
        # auto-detect per channel whether the daemon is pushing: the first
        # webhook on a channel makes the webhook authoritative for it, and until
        # then (or if never configured) the filament_detect read path is used.
        # filament_detect still owns tag-removal/clear (the daemon's
        # tag_not_present event only goes there), so that keeps working too.
        self._webhook_channels_seen: set = set()
        # webhook_grace: seconds to defer a colour-lossy filament_detect read of
        # a NEW tag so the OpenRFID full-colour webhook (if the daemon pushes)
        # can land first and be the authoritative read — avoids a "misread" where
        # filament_detect's primary-only colour wins the first scan of a tag.
        # 0 = off (no deferral). Only ever delays the first read on a channel:
        # once a webhook is seen, filament_detect reads are suppressed anyway.
        # Set ~1.0 if you use the OpenRFID webhook. Harmless (just adds latency)
        # if the daemon never pushes.
        self._webhook_grace = config.getfloat('webhook_grace', 0.0, minval=0.0)
        self._pending_defer: Dict[int, list] = {}  # channel -> uid awaiting grace
        try:
            webhooks = self.printer.lookup_object('webhooks')
            webhooks.register_endpoint('afc/u1_rfid', self._handle_webhook_scan)
        except Exception as e:
            self.logger.warning(
                f"AFC_U1_rfid: failed to register webhook endpoint: {e}")
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_ready(self):
        """Resolve configured lanes against AFC and start polling. Replaces the
        old AFC_prep._init_u1_rfid wiring that lived in the (now upstream) core.
        """
        self.afc = self.printer.lookup_object('AFC', None)
        if self.afc is None:
            self.logger.warning("AFC_U1_rfid: AFC not loaded; reader disabled")
            return
        self.logger = self.afc.logger
        for lane_name, channel in self._cfg_channels.items():
            lane = self._resolve_lane(lane_name)
            if lane is None:
                self.logger.warning(
                    f"U1 RFID: configured lane '{lane_name}' not found in AFC "
                    f"(neither a lane name nor a single-lane extruder)")
                continue
            if lane_name in self._cfg_scanners:
                # Mark the lane so any scanner-aware code (incl. bridge hooks
                # that read lane.spool_scanner) sees it.
                try:
                    lane.spool_scanner = True
                except Exception:
                    pass
            self.register_lane(lane, channel)
        # Standalone scanner channels: no lane to resolve — register the channel
        # directly with a None lane so the poll/callback path handles it.
        for channel in self._cfg_scanner_channels:
            self._channel_to_lane[channel] = None
            self._last_uid[channel] = None
            self._consecutive_failures[channel] = 0
        self.start()
        self._patch_scanner_rfid_update()

    def _patch_scanner_rfid_update(self):
        """Stop a spool-scanner read from overwriting the U1 display (and
        resetting flow K) for the extruder the antenna sits on.

        The U1's native ``print_task_config._rfid_filament_info_update_cb``
        blindly writes a scanned tag into print_task_config for the reader's
        channel and runs FLOW_RESET_K — clobbering the AFC-loaded lane's filament
        shown for that physical extruder. For our standalone scanner channels we
        suppress that callback, so the loaded lane's data (which AFC writes on
        load) stays on the display and its flow K is preserved. Self-contained:
        depends on nothing but the U1's own objects. No-ops on non-U1.
        """
        ptc = self.printer.lookup_object("print_task_config", None)
        fd = self.printer.lookup_object("filament_detect", None)
        if ptc is None or fd is None \
                or not hasattr(fd, "_notify_data_update_cb"):
            return
        original_cb = getattr(ptc, "_rfid_filament_info_update_cb", None)
        if original_cb is None:
            return
        scanner_channels = set(self._cfg_scanner_channels)
        if not scanner_channels:
            return

        def patched_rfid_cb(channel, info, is_clear=False):
            """filament_detect callback wrapper that protects scanner channels.

            Suppresses the native print_task_config write for configured scanner
            channels (keeping the loaded lane's display + flow K); every other
            channel passes straight through to the original callback unchanged.

            :param channel: U1 filament_detect channel index.
            :param info: RFID info dict for the channel.
            :param is_clear: True when the event signals tag removal/clear.
            """
            # Suppress the native write for scanner channels — keep the loaded
            # lane's display + flow K; everything else passes through unchanged.
            if channel in scanner_channels:
                return
            original_cb(channel, info, is_clear)

        for i, cb in enumerate(fd._notify_data_update_cb):
            if cb == original_cb:
                fd._notify_data_update_cb[i] = patched_rfid_cb
                self.logger.info(
                    "U1 RFID: protecting scanner channels %s from U1 display "
                    "overwrite" % sorted(scanner_channels))
                return
        self.logger.warning(
            "U1 RFID: could not locate print_task_config RFID callback to patch")

    def _resolve_lane(self, name):
        """Resolve a configured name to a lane object.

        Tries the AFC lane registry first (lane name). For individual-extruder
        tool setups the name is often the *extruder* name, so fall back to the
        single lane driving that extruder — matching either the AFC_extruder
        section name (extruder_obj.name) or the toolhead extruder name
        (extruder_obj.th_extruder_name, added upstream in v1.1.22). Returns None
        if it can't be resolved unambiguously, logging what IS available so a
        config mismatch is obvious.

        :param name: configured name — an AFC lane name or an extruder name.
        :return: the resolved AFCLane, or None if it can't be resolved
            unambiguously.
        """
        lane = self.afc.lanes.get(name)
        if lane is not None:
            return lane

        def _ext_names(l):
            """Return the set of extruder names associated with a lane.

            :param l: an AFC lane object.
            :return set: the lane's AFC_extruder section name and toolhead
                extruder name (th_extruder_name), for matching by either.
            """
            e = getattr(l, 'extruder_obj', None)
            return {getattr(e, 'name', None),
                    getattr(e, 'th_extruder_name', None)}

        matches = [l for l in self.afc.lanes.values() if name in _ext_names(l)]
        # Also consult the extruder registry directly, in case a lane's
        # extruder_obj linkage isn't reflected in the scan above.
        if not matches:
            tools = getattr(self.afc, 'tools', {}) or {}
            ext = tools.get(name)
            if ext is None:
                for e in tools.values():
                    if name in {getattr(e, 'name', None),
                                getattr(e, 'th_extruder_name', None)}:
                        ext = e
                        break
            if ext is not None:
                matches = list(getattr(ext, 'lanes', {}).values())

        if len(matches) == 1:
            return matches[0]
        if len(matches) > 1:
            self.logger.warning(
                f"U1 RFID: '{name}' matches {len(matches)} lanes on that "
                f"extruder — use the specific lane name instead")
            return None

        # Nothing matched — surface what's registered so the user can correct
        # the config. (Upstream v1.1.22 made standalone lanes opt-in via
        # 'standalone: True' instead of inferring from the lane name, so a
        # standalone toolhead lane that's missing the flag may not be here.)
        avail_lanes = sorted(self.afc.lanes.keys())
        avail_ext = sorted(
            {n for l in self.afc.lanes.values() for n in _ext_names(l) if n})
        self.logger.warning(
            f"U1 RFID: '{name}' resolved to no lanes. Available lanes="
            f"{avail_lanes}; extruders={avail_ext}. If '{name}' is a standalone "
            f"toolhead, ensure its [AFC_stepper] has 'standalone: True' and the "
            f"[AFC_extruder {name}] section exists.")
        return None

    def register_lane(self, lane: AFCLane, channel: int):
        """Register a lane to monitor a specific filament_detect channel.

        :param lane: AFC lane instance to associate with the channel.
        :param channel: U1 filament_detect channel index.
        :return: None
        """
        self._lane_channel_map[lane.name] = channel
        self._lane_objects[lane.name] = lane
        self._last_uid[channel] = None
        self._channel_to_lane[channel] = lane.name
        self._consecutive_failures[channel] = 0

    def start(self):
        """Start polling filament_detect for RFID data.

        :return: None
        """
        if not self._lane_channel_map and not self._cfg_scanner_channels:
            return
        self._gcode = self.afc.gcode
        channels = list(self._lane_channel_map.items())
        self._scanner_channels = {ch for name, ch in channels
                                   if name in self._cfg_scanners}
        self._scanner_channels |= self._cfg_scanner_channels
        if channels:
            self.logger.info(
                f"U1 RFID: monitoring {len(channels)} lane channel(s): "
                + ", ".join(f"{name}=ch{ch}" for name, ch in channels))
        if self._cfg_scanner_channels:
            self.logger.info(
                "U1 RFID: standalone spool scanner channel(s): "
                + ", ".join(f"ch{ch}" for ch in sorted(self._cfg_scanner_channels)))
        lane_scanner_names = [name for name, ch in channels
                              if ch in self._scanner_channels]
        if lane_scanner_names:
            self.logger.info(
                f"U1 RFID: lane-attached scanner(s): {', '.join(lane_scanner_names)}")
        self._try_attach_filament_detect()
        self._poll_timer = self.reactor.register_timer(
            self._poll_cb, self.reactor.monotonic() + POLL_INTERVAL)

    def _try_attach_filament_detect(self) -> bool:
        """Look up filament_detect and register the push callback.

        :return: True if filament_detect is available, False otherwise.
        """
        if self._filament_detect is not None:
            return True
        fd = self.printer.lookup_object("filament_detect", None)
        if fd is None:
            return False
        self._filament_detect = fd
        # Log the actual API surface once — RFID integration breakages are
        # almost always a method-name mismatch against the U1's filament_detect.
        known = ('_notify_data_update_cb', 'register_cb_2_update_filament_info',
                 'get_a_filament_info', 'get_all_filament_info', 'get_status',
                 'update_filament_info', 'request_update')
        present = [n for n in known if hasattr(fd, n)]
        self.logger.info(
            "U1 RFID: filament_detect attached (api: %s)"
            % (", ".join(present) or "none recognized"))
        self._register_fd_callback(fd)
        return True

    def _register_fd_callback(self, fd) -> None:
        """Register our push callback with filament_detect.

        register_cb_2_update_filament_info() is the PROVEN path — our working
        pre-refactor forks used exactly this, paired with FILAMENT_DT_UPDATE to
        trigger the read. Fall back to appending to the raw _notify_data_update_cb
        list (where print_task_config's own RFID callback lives) for other
        firmware revs.

        :param fd: the filament_detect Klipper object to register the callback on.
        """
        if self._fd_cb_registered:
            return
        if hasattr(fd, 'register_cb_2_update_filament_info'):
            try:
                fd.register_cb_2_update_filament_info(
                    self._on_filament_info_update)
                self._fd_cb_registered = True
                self.logger.info(
                    "U1 RFID: push callback registered via "
                    "register_cb_2_update_filament_info")
                return
            except Exception as e:
                self.logger.warning(
                    f"U1 RFID: failed to register info callback: {e}")
        cb_list = getattr(fd, '_notify_data_update_cb', None)
        if isinstance(cb_list, list):
            if self._on_filament_info_update not in cb_list:
                cb_list.append(self._on_filament_info_update)
            self._fd_cb_registered = True
            self.logger.info(
                "U1 RFID: push callback registered via _notify_data_update_cb")
            return
        self.logger.warning(
            "U1 RFID: no recognized filament_detect push-callback API; "
            "scanner will rely on polling only")

    def _on_filament_info_update(self, *args):
        """Callback fired by filament_detect with (channel, info_dict, official).

        :param args: Variable positional arguments forwarded by filament_detect.
        :return: None
        """
        if len(args) >= 2 and isinstance(args[0], int) and isinstance(args[1], dict):
            channel = args[0]
            info = args[1]
            # Registered channels map to a lane name, or None for standalone
            # scanner channels — both are valid; dispatch on membership.
            if channel in self._channel_to_lane:
                lane_name = self._channel_to_lane.get(channel)
                try:
                    self._check_channel(lane_name, channel, info=info)
                except Exception as e:
                    self.logger.warning(
                        f"U1 RFID: _on_filament_info_update error ch{channel}: {e}")
            return
        for lane_name, channel in self._lane_channel_map.items():
            try:
                self._check_channel(lane_name, channel)
            except Exception as e:
                self.logger.warning(
                    f"U1 RFID: _on_filament_info_update error {lane_name}: {e}")

    def stop(self):
        """Stop polling.

        :return: None
        """
        if self._poll_timer is not None:
            self.reactor.update_timer(self._poll_timer, self.reactor.NEVER)

    def _trigger_channel_update(self, channel: int) -> bool:
        """Trigger a fresh read from hardware for a scanner channel.
        Returns True on success, False on failure.

        :param channel: U1 filament_detect channel index.
        :return: True on success, False on failure.
        """
        fd = self._filament_detect
        if fd is None:
            return False
        # FILAMENT_DT_UPDATE is the PROVEN trigger (our pre-refactor forks used
        # exactly this for scanner channels): it makes the U1 read the channel
        # AND fire the registered notify callback, which is how a scan reaches
        # _on_filament_info_update -> _check_channel. Prefer it over
        # fd.update_filament_info(), which on this firmware refreshes internal
        # state WITHOUT notifying, so the read is silently dropped.
        try:
            self._gcode.run_script_from_command(
                f"FILAMENT_DT_UPDATE CHANNEL={channel}")
            return True
        except Exception as e:
            self.logger.warning(
                f"U1 RFID: FILAMENT_DT_UPDATE failed ch{channel}: {e}")
        if hasattr(fd, 'update_filament_info'):
            try:
                fd.update_filament_info(channel)
                return True
            except Exception:
                pass
        if hasattr(fd, 'request_update'):
            try:
                fd.request_update(channel)
                return True
            except Exception:
                pass
        return False

    def _poll_cb(self, eventtime):
        """Periodic check for new RFID data on registered channels.

        :param eventtime: Current reactor monotonic time.
        :return: Next poll time as a float.
        """
        if not self._try_attach_filament_detect():
            return eventtime + _BACKOFF_INTERVAL

        for ch in self._scanner_channels:
            if not self._trigger_channel_update(ch):
                self._consecutive_failures[ch] = \
                    self._consecutive_failures.get(ch, 0) + 1
                if self._consecutive_failures[ch] == _MAX_CONSECUTIVE_FAILURES:
                    self.logger.error(
                        f"U1 RFID: ch{ch} failed {_MAX_CONSECUTIVE_FAILURES} "
                        f"times consecutively, backing off")
                    self._backed_off = True
            else:
                self._consecutive_failures[ch] = 0

        # Standalone scanner channels (no lane) aren't in _lane_channel_map, so
        # the lane loop below never reads them. Without a polling check here they
        # depend entirely on filament_detect's push callback firing — which it
        # may not for a held spool (only-on-change pushes, or update triggers
        # that don't call back). Poll them directly so a scan is always read.
        for ch in self._cfg_scanner_channels:
            try:
                self._check_channel(None, ch)
            except Exception as e:
                self.logger.warning(
                    f"U1 RFID: poll error on scanner ch{ch}: {e}")

        for lane_name, channel in self._lane_channel_map.items():
            try:
                self._check_channel(lane_name, channel)
            except Exception as e:
                self.logger.warning(
                    f"U1 RFID: poll error on {lane_name} ch{channel}: {e}")

        if self._backed_off:
            self._backoff_cycles += 1
            if self._backoff_cycles >= _BACKOFF_RESET_CYCLES:
                self._backed_off = False
                self._backoff_cycles = 0
                for ch in self._scanner_channels:
                    self._consecutive_failures[ch] = 0
                self.logger.info("U1 RFID: backoff reset, retrying normal polling")
            else:
                all_recovered = all(
                    self._consecutive_failures.get(ch, 0) < _MAX_CONSECUTIVE_FAILURES
                    for ch in self._scanner_channels)
                if all_recovered:
                    self._backed_off = False
                    self._backoff_cycles = 0
            return eventtime + _BACKOFF_INTERVAL
        return eventtime + POLL_INTERVAL

    _LOCKED_STATES = frozenset({
        "Loaded", "Tooled", "Tool Loaded", "Tool Loading", "Tool Unloading",
        "HUB Loading",
    })

    def _send_lane_data(self, lane):
        """Push lane data to moonraker, guarded.

        Upstream AFCLane.send_lane_data() dereferences self.afc.moonraker with
        no None check. A tag can be read before moonraker finishes connecting
        (it connects async after klippy:ready), so skip the push until it's up —
        the data is re-sent on the next read / save_vars.

        :param lane: the AFC lane whose data to push to moonraker.
        """
        if getattr(self.afc, 'moonraker', None) is None:
            return
        try:
            lane.send_lane_data()
        except Exception as e:
            self.logger.debug(
                f"U1 RFID: send_lane_data skipped for "
                f"{getattr(lane, 'name', '?')}: {e}")

    def _handle_webhook_scan(self, web_request):
        """Receive a full tag read pushed by the OpenRFID daemon.

        The daemon's GenericFilament carries the COMPLETE colour list (e.g. a
        dual-colour spool's real second colour), which filament_detect drops.
        Re-pack it into the same info schema _map_to_slot_info expects — with
        COLOR_NUMS = the true colour count and RGB_n = each ARGB colour — then
        run it through the normal scan path as the authoritative source.

        Expected JSON body (see the [webhook_exporter] config block):
            {"channel": int, "manufacturer": str, "type": str,
             "sub_type": str, "colors": [argb_int, ...],
             "hotend_min_temp": int, "hotend_max_temp": int,
             "bed_temp": int, "weight_grams": int, "card_uid": [int, ...],
             "manufacturing_date": str}   # -> Spoolman lot_nr

        :param web_request: Klipper webhook request carrying the JSON body
            described above (read via its get_int/get accessors).
        """
        try:
            channel = web_request.get_int('channel')
        except Exception:
            return
        if channel not in self._channel_to_lane:
            return  # not a channel we monitor
        colors = web_request.get('colors', [])
        if not isinstance(colors, (list, tuple)):
            colors = []
        info = {
            'VENDOR': web_request.get('manufacturer', ''),
            'MAIN_TYPE': web_request.get('type', ''),
            'SUB_TYPE': web_request.get('sub_type', ''),
            'HOTEND_MIN_TEMP': web_request.get_int('hotend_min_temp', 0),
            'HOTEND_MAX_TEMP': web_request.get_int('hotend_max_temp', 0),
            'BED_TEMP': web_request.get_int('bed_temp', 0),
            'WEIGHT': web_request.get_int('weight_grams', 0),
            'COLOR_NUMS': len(colors),
            'CARD_UID': web_request.get('card_uid', None),
            'MF_DATE': web_request.get('manufacturing_date', ''),
        }
        for idx, c in enumerate(colors, start=1):
            try:
                info['RGB_%d' % idx] = int(c)
            except (ValueError, TypeError):
                continue
        # First webhook on this channel: the daemon IS pushing full data, so
        # make the webhook authoritative for it from now on (future colour-lossy
        # filament_detect reads are suppressed by the source check in
        # _check_channel). Do NOT clear _last_uid here: if a filament_detect read
        # already processed this exact tag, the UID dedup must still suppress this
        # duplicate webhook so the same scan isn't synced/created twice.
        if channel not in self._webhook_channels_seen:
            self._webhook_channels_seen.add(channel)
        lane_name = self._channel_to_lane.get(channel)
        try:
            self._check_channel(lane_name, channel, info=info, source='webhook')
        except Exception as e:
            self.logger.warning(
                f"U1 RFID: webhook scan error ch{channel}: {e}")

    def _check_channel(self, lane_name: str, channel: int, info: dict = None,
                       source: str = 'poll'):
        """Check a single channel for new or changed RFID data.

        For spool_scanner lanes the spool is assigned directly to the lane
        (bypassing the ``next_spool_id`` staging mechanism) so that Spoolman
        flow K sync triggers immediately on scan.

        :param lane_name: AFC lane name mapped to this RFID channel.
        :param channel: U1 filament_detect channel index.
        :param info: Pre-fetched RFID info dict, or *None* to read live.
        :param source: 'poll'/'push' (filament_detect) or 'webhook'.
        :return: None
        """
        if info is None:
            if self._filament_detect is None:
                return
            info = self._get_channel_info(channel)
        # Standalone scanner channel: no lane; the scan stages next_spool_id.
        scanner_only = channel in self._cfg_scanner_channels
        lane = None if scanner_only else self._lane_objects.get(lane_name)
        is_scanner = scanner_only or (
            lane is not None and getattr(lane, 'spool_scanner', False))

        if info is None:
            return

        card_uid = info.get("CARD_UID")
        if not card_uid or card_uid == 0:
            if self._last_uid.get(channel) not in (None, 0):
                if not is_scanner:
                    self._last_uid[channel] = 0
                    if lane is not None and getattr(lane, "status", "") not in self._LOCKED_STATES:
                        self._clear_lane(lane, lane_name)
                # Scanner channels intentionally keep _last_uid set after a
                # scan: reading a spool stages its next_spool_id, so the same
                # spool must NOT re-fire while/after it's presented. A different
                # spool (new uid) still triggers via the dedup check below.
            return

        # A tag is present. Once the OpenRFID daemon has pushed a webhook for
        # this channel, the webhook is the authoritative (full-colour) source —
        # ignore the colour-lossy filament_detect read here. (Removal/clear
        # above still runs, since that only arrives via filament_detect.)
        if source != 'webhook' and channel in self._webhook_channels_seen:
            return

        if card_uid == self._last_uid.get(channel):
            return

        if not scanner_only and lane is None:
            return

        if not is_scanner and getattr(lane, "status", "") in self._LOCKED_STATES:
            return

        # Webhook-preference grace: defer a colour-lossy filament_detect ('poll')
        # read of a NEW tag so the full-colour webhook can land first. Don't set
        # _last_uid yet (so an arriving webhook isn't deduped); track the pending
        # defer so repeat polls don't re-arm it. _grace_expired falls back to the
        # filament_detect read only if no webhook arrived. ('poll-final' bypasses
        # this so the deferred read actually processes.)
        if (self._webhook_grace > 0 and source == 'poll'
                and channel not in self._webhook_channels_seen):
            if self._pending_defer.get(channel) != card_uid:
                self._pending_defer[channel] = card_uid
                self.reactor.register_callback(
                    lambda et, ln=lane_name, ch=channel, uid=card_uid:
                        self._grace_expired(ln, ch, uid),
                    self.reactor.monotonic() + self._webhook_grace)
            return

        self._last_uid[channel] = card_uid

        # Dump the raw tag dict so field-mapping issues (colour, temps, vendor)
        # are diagnosable without guessing filament_detect's schema.
        self.logger.debug(f"U1 RFID: ch{channel} raw tag info: {info}")

        main_type = info.get("MAIN_TYPE", "")
        if not main_type or main_type.upper() == "NONE":
            return

        slot_info = self._map_to_slot_info(info)
        brand = slot_info.get('brand', '')
        material = slot_info.get('material', '')
        color = slot_info.get('color_hex', '')
        multi_color = slot_info.get('multi_color', [color] if color else [])
        tag_desc = f"{brand} {material}".strip() or "Unknown"
        clabel = color_label(multi_color)
        if clabel:
            tag_desc += f" ({clabel})"

        if is_scanner:
            self.logger.info(f"U1 RFID: spool scanned — {tag_desc}")
            # Scanner channels have no lane, so the lane-based auto-create
            # lookup doesn't apply — use the configured scanner default.
            allow_create = (self._scanner_auto_create if scanner_only
                            else get_auto_spoolman_create(
                                lane, self._lane_auto_create))
            sync_rfid_to_spoolman(
                self.afc, lane, slot_info, self.logger, "U1 RFID",
                allow_create=allow_create, set_next=True)
            self._notify_scan(brand, material, color, slot_info,
                              lane_name=(lane_name or f"scanner-ch{channel}"),
                              is_scanner=True)
            return

        self.logger.info(f"U1 RFID: tag detected on {lane_name} — {tag_desc}")
        if getattr(lane, "spool_id", None) not in (None, "", 0):
            self.afc.spool.set_spoolID(lane, "")
        apply_filament_defaults(lane, slot_info)
        allow_create = get_auto_spoolman_create(lane, self._lane_auto_create)
        sync_rfid_to_spoolman(
            self.afc, lane, slot_info, self.logger, "U1 RFID",
            allow_create=allow_create)
        self._notify_scan(brand, material, color, slot_info,
                          lane_name=lane_name)
        self._send_lane_data(lane)
        self.afc.save_vars()
        if getattr(lane, 'tool_loaded', False):
            self.printer.send_event("afc:tool_loaded", lane)

    def _grace_expired(self, lane_name, channel, card_uid):
        """webhook_grace timer: process the deferred filament_detect read only if
        no webhook arrived for the channel during the grace window.

        :param lane_name: AFC lane name for the channel (None for scanner-only).
        :param channel: U1 filament_detect channel index.
        :param card_uid: the tag UID the deferral was armed for; the read is
            skipped if a newer tag has since superseded it.
        """
        if self._pending_defer.get(channel) != card_uid:
            return  # superseded by a newer tag, or already cleared
        self._pending_defer.pop(channel, None)
        if channel in self._webhook_channels_seen:
            return  # a webhook landed during the grace and handled the tag
        try:
            self._check_channel(lane_name, channel, source='poll-final')
        except Exception as e:
            self.logger.warning(
                f"U1 RFID: deferred read error ch{channel}: {e}")

    def _clear_lane(self, lane, lane_name: str):
        """Clear RFID data from a lane when tag is removed.

        :param lane: AFC lane instance to clear.
        :param lane_name: Name of the lane being cleared.
        :return: None
        """
        lane.material = ""
        lane.color = ""
        if getattr(lane, "spool_id", None) not in (None, "", 0):
            try:
                self.afc.spool.set_spoolID(lane, "")
            except Exception as e:
                self.logger.warning(
                    f"U1 RFID: failed to clear spool_id on {lane_name}: {e}")
        self._send_lane_data(lane)
        self.afc.save_vars()

    def _get_channel_info(self, channel: int) -> Optional[dict]:
        """Read filament info for a channel from filament_detect.

        :param channel: U1 filament_detect channel index.
        :return: RFID info dict for the channel, or None if unavailable.
        """
        fd = self._filament_detect
        if hasattr(fd, 'get_a_filament_info'):
            try:
                info = fd.get_a_filament_info(channel)
                if isinstance(info, dict):
                    return info
            except Exception:
                pass
        if hasattr(fd, 'get_all_filament_info'):
            try:
                all_info = fd.get_all_filament_info()
                if isinstance(all_info, (list, tuple)) and channel < len(all_info):
                    entry = all_info[channel]
                    if isinstance(entry, dict):
                        return entry
                elif isinstance(all_info, dict):
                    entry = all_info.get(channel) or all_info.get(str(channel))
                    if isinstance(entry, dict):
                        return entry
            except Exception:
                pass
        if hasattr(fd, 'get_status'):
            try:
                status = fd.get_status()
                if isinstance(status, dict):
                    info_list = status.get('info')
                    if info_list and channel < len(info_list):
                        entry = info_list[channel]
                        if isinstance(entry, dict) and entry.get("CARD_UID"):
                            return entry
            except Exception:
                pass
        return None

    def _tag_color_count(self, info: dict) -> Optional[int]:
        """Return the tag's declared colour count, or None if not present.

        The U1's filament_detect schema isn't fully known here, so match any
        key that means "colour count" (contains COLOR/COLOUR and COUNT/NUM/NUMS)
        rather than hard-coding one name. The OpenRFID Bambu processor decodes
        this as "Color Count", so the forwarded info dict should expose it.

        :param info: raw RFID info dict from filament_detect / the webhook.
        :return: the declared colour count (>= 1), or None if no such field.
        """
        for k, v in info.items():
            ku = str(k).upper()
            if (("COLOR" in ku or "COLOUR" in ku)
                    and ("COUNT" in ku or "NUM" in ku)):
                try:
                    n = int(v)
                except (ValueError, TypeError):
                    continue
                if n >= 1:
                    return n
        return None

    def _map_to_slot_info(self, info: dict) -> dict:
        """Map filament_detect fields to AFC RFID slot_info format.

        :param info: Raw RFID info dict from filament_detect.
        :return: Normalized slot info dict for AFC use.
        """
        # Gather colours in tag order (RGB_1, RGB_2, ...), masking the ARGB
        # alpha byte (-> RRGGBB).
        ordered = []
        rgb_keys = sorted((k for k in info if re.fullmatch(r"RGB_\d+", str(k))),
                          key=lambda k: int(str(k).split("_")[1]))
        for key in rgb_keys:
            raw = info.get(key)
            if raw is None or raw == "":
                continue
            try:
                raw_int = int(raw)
            except (ValueError, TypeError):
                continue
            ordered.append((raw_int, f"{raw_int & 0xFFFFFF:06x}"))

        # How many colours does the tag actually carry? Prefer the tag's own
        # colour-count field (authoritative — correctly handles e.g. a genuine
        # white second colour). Only when it's absent do we fall back to dropping
        # the U1's unused-slot white sentinel (0xFFFFFFFF) on secondary slots,
        # which otherwise makes a single-colour spool look dual-colour.
        color_count = self._tag_color_count(info)
        multi_color = []
        if color_count is not None and color_count >= 1:
            for _raw_int, hx in ordered[:color_count]:
                if hx not in multi_color:
                    multi_color.append(hx)
            _src = f"tag count={color_count}"
        else:
            for raw_int, hx in ordered:
                if multi_color and raw_int == 0xFFFFFFFF:
                    continue  # unused secondary slot, not a real colour
                if hx not in multi_color:
                    multi_color.append(hx)
            _src = "no tag count field; white-sentinel heuristic"
        color_hex = multi_color[0] if multi_color else ""
        self.logger.debug(
            f"U1 RFID: parsed {len(multi_color)} colour(s) {multi_color} "
            f"from RGB slots {[hx for _, hx in ordered]} ({_src})")
        ext_max = info.get("HOTEND_MAX_TEMP")
        ext_min = info.get("HOTEND_MIN_TEMP")
        bed_max = info.get("BED_TEMP")
        sku_raw = info.get("SKU", "")
        sku = "" if (not sku_raw or sku_raw == 0) else str(sku_raw)
        vendor = info.get("VENDOR", "")
        if vendor.upper() == "NONE":
            vendor = ""
        if ext_max and ext_min:
            ext_temp = (int(ext_max) + int(ext_min)) // 2
        elif ext_max:
            ext_temp = int(ext_max)
        else:
            ext_temp = None
        return {
            "material": info.get("MAIN_TYPE", ""),
            "color_hex": color_hex,
            "multi_color": multi_color,
            "is_dual_color": len(multi_color) > 1,
            "sku": sku,
            "brand": vendor,
            "sub_type": info.get("SUB_TYPE", ""),
            "diameter": 1.75,
            "extruder_temp": ext_temp,
            "bed_temp": int(bed_max) if bed_max else None,
            # Tag metadata for Spoolman: manufacturing date -> lot_nr, card UID
            # -> card_uids extra field. (MF_DATE is reliable on the OpenRFID
            # webhook path; the filament_detect path often reports it unset.)
            "mfg_date": self._fmt_mfg_date(info.get("MF_DATE")),
            "uid": self._fmt_uid(info.get("CARD_UID")),
        }

    @staticmethod
    def _fmt_mfg_date(raw):
        """Normalize a tag manufacturing date to a clean string (lot_nr), or
        None if unset. Accepts YYYYMMDD (filament_detect) or ISO YYYY-MM-DD
        (OpenRFID webhook); treats epoch/1970 as 'unset'.

        :param raw: the tag's raw manufacturing-date value (str/int/None).
        :return: a normalised 'YYYY-MM-DD' string (Spoolman lot_nr), or None.
        """
        if not raw:
            return None
        s = str(raw).strip()
        if not s or s.startswith("1970") or s in ("0", "00000000"):
            return None
        if len(s) == 8 and s.isdigit():
            return f"{s[0:4]}-{s[4:6]}-{s[6:8]}"
        return s

    @staticmethod
    def _fmt_uid(raw):
        """Format a card UID (list of byte ints, e.g. [123,240,175,255]) as an
        uppercase hex string ('7BF0AFFF'); pass through a non-empty string.

        :param raw: a card UID as a list/tuple of byte ints, or a string.
        :return: the formatted uppercase-hex UID string, or None if empty.
        """
        if not raw:
            return None
        if isinstance(raw, (list, tuple)):
            try:
                return "".join("%02X" % (int(b) & 0xFF) for b in raw)
            except (ValueError, TypeError):
                return None
        s = str(raw).strip()
        return s or None

    def _notify_scan(self, brand: str, material: str, color: str,
                     slot_info: dict, lane_name: str = "",
                     is_scanner: bool = False):
        """Send a user-visible notification when RFID reads a spool.

        Sends to three targets: Klipper console (respond_info), Mainsail/
        Octoprint (action:prompt), and U1 factory display (exception_manager).

        :param brand: Filament brand name.
        :param material: Filament material type (e.g. "PLA").
        :param color: Hex color string (without '#').
        :param slot_info: Full slot info dict from RFID tag.
        :param lane_name: Lane name; empty string if unknown.
        :param is_scanner: True when this is a spool_scanner read.
        :return: None
        """
        try:
            cname = color_name(color) if color else ""
            ext = slot_info.get("extruder_temp")
            bed = slot_info.get("bed_temp")
            raw = self.logger.raw
            lane = self._lane_objects.get(lane_name) if lane_name else None
            spool_id = getattr(lane, "spool_id", None) if lane else None

            if is_scanner:
                title = "Spool Scanned"
                header = "Spool scanned on %s:" % lane_name if lane_name else "Spool scanned:"
            else:
                title = "Spool Loaded — %s" % lane_name if lane_name else "Spool Loaded"
                header = "Spool loaded on %s:" % lane_name if lane_name else "Spool loaded:"

            lines = [header]
            if brand:
                lines.append(f"  Brand: {brand}")
            if material:
                lines.append(f"  Material: {material}")
            if color:
                label = f"{cname} (#{color})" if cname else f"#{color}"
                lines.append(f"  Color: {label}")
            if ext:
                lines.append(f"  Nozzle temp: {ext}°C")
            if bed:
                lines.append(f"  Bed temp: {bed}°C")
            if spool_id:
                lines.append(f"  Spoolman ID: {spool_id}")
            self.afc.gcode.respond_info("\n".join(lines))

            prompt_lines = []
            if brand:
                prompt_lines.append(f"Brand: {brand}")
            if material:
                prompt_lines.append(f"Material: {material}")
            if color:
                label = f"{cname} (#{color})" if cname else f"#{color}"
                prompt_lines.append(f"Color: {label}")
            if ext:
                prompt_lines.append(f"Nozzle: {ext}°C")
            if bed:
                prompt_lines.append(f"Bed: {bed}°C")
            if spool_id:
                prompt_lines.append(f"Spoolman ID: {spool_id}")
            raw(f"// action:prompt_begin {title}")
            for pl in prompt_lines:
                raw(f"// action:prompt_text {pl}")
            raw("// action:prompt_footer_button OK|RESPOND TYPE=command MSG=action:prompt_end|info")
            raw("// action:prompt_show")

            self.reactor.register_callback(
                lambda e: self.logger.raw("// action:prompt_end"),
                self.reactor.monotonic() + 10.0)

            if is_scanner:
                em = self.printer.lookup_object("exception_manager", None)
                if em is not None:
                    parts = []
                    if brand:
                        parts.append(brand)
                    if material:
                        parts.append(material)
                    if cname:
                        parts.append(cname)
                    msg = "%s: %s" % (title, " ".join(parts)) if parts else title
                    channel = self._lane_channel_map.get(lane_name, 0)
                    em.raise_exception_async(
                        id=529, index=channel, code=99,
                        message=msg, oneshot=1, level=1)
        except Exception as e:
            self.logger.warning(f"U1 RFID: notification error: {e}")

    def force_read(self, lane_name: str):
        """Force an RFID re-read for a specific lane.

        :param lane_name: Name of the lane to re-read.
        :return: None
        """
        channel = self._lane_channel_map.get(lane_name)
        if channel is None:
            return
        self._last_uid[channel] = None
        if not self._trigger_channel_update(channel):
            self.logger.warning(
                f"U1 RFID: force_read failed to trigger update for {lane_name}")
            return
        deadline = self.reactor.monotonic() + _FORCE_READ_TIMEOUT
        while self.reactor.monotonic() < deadline:
            info = self._get_channel_info(channel)
            if info is not None and info.get("CARD_UID"):
                self._check_channel(lane_name, channel, info=info)
                return
            self.reactor.pause(
                self.reactor.monotonic() + _FORCE_READ_POLL_STEP)
        self._check_channel(lane_name, channel)


def load_config(config):
    """Klipper config hook: instantiate the U1 RFID reader.

    :param config: Klipper config wrapper for the [AFC_U1_rfid] section.
    :return: the AFC_U1_RFID instance.
    """
    return AFC_U1_RFID(config)
