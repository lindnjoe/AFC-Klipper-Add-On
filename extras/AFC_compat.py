# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# AFC compatibility shims.
#
# Runtime guards that let our serial-driven units (ACE, OpenAMS) run on the
# FROZEN upstream AFC core without editing upstream files. They re-express
# behaviour our deviated fork had (steppermless [AFC_lane] lanes on virtual
# hubs, two-phase unload) against upstream's classes.
#
# Each shim is idempotent (a flag on the target class) and safe to apply from
# multiple units — whichever loads first wins. Call apply_compat_patches() from
# each unit module AT IMPORT TIME, so it runs before the [AFC_lane ...] sections
# init (standard AFC config has the unit section before its lanes).
#
# Shims:
#   1. AFC_lane     - skip homing-endstop setup on a 'virtual' hub pin
#                     (upstream tries setup_pin('endstop','virtual') -> crash).
#   2. AFC_buffer   - skip fault detection for steppermless [AFC_lane] lanes
#                     (no extruder_stepper -> no movement to measure -> false fault).
#   3. AFC_hub      - virtual-hub state from the driven hub HES (switch_pin_callback),
#                     not the lane F1S/load sensor (any(lane.raw_load_state)).
#   4. AFC.load/unload_sequence - run the shared toolhead phase (heat/cut/park/
#                     form_tip + post_unload_macro) around a custom_(un)load_cmd,
#                     which upstream skips for custom commands.
#   5. afcFunction.cmd_CALIBRATE_AFC - let steppermless serial units run their
#                     self-contained bowden/PTFE calibration (upstream's BOWDEN=
#                     guard requires a tool_start sensor and aborts otherwise).
#   6. AFC_lane.handle_load_runout - generalize the load/runout callback for
#                     serial-polled units (guard the optional physical switch,
#                     gate on serial types, route runout via the unit handler).
#   7. afcUnit       - base on_filament_insert/on_filament_remove hooks our
#                     serial units call via super() (insert fires afc:lane_inserted).
#   8. afc_hub.handle_connect - a virtual hub only needs a lane's load sensor
#                     when that lane also has a prep sensor (fully sensorless
#                     serial lanes are valid; upstream rejects any missing load).

from __future__ import annotations


def _patch_afc_lane_virtual_hub():
    """1. AFC_lane: don't set up a homing endstop on a 'virtual' hub pin."""
    try:
        from extras import AFC_lane as _lane_mod
    except Exception:
        return
    LaneCls = getattr(_lane_mod, 'AFCLane', None)
    if LaneCls is None or getattr(LaneCls, '_afc_virtual_hub_patched', False):
        return
    _orig = LaneCls._set_homing_endstop

    def _guarded(self, query_endstops, ppins, pin, name):
        if pin is not None and str(pin).strip().lower() == "virtual":
            return  # virtual hub has no physical pin; no homing endstop
        return _orig(self, query_endstops, ppins, pin, name)

    LaneCls._set_homing_endstop = _guarded
    LaneCls._afc_virtual_hub_patched = True


def _patch_afc_buffer_steppermless():
    """2. AFC_buffer: skip the fault timer for lanes with no extruder_stepper."""
    try:
        from extras import AFC_buffer as _buf_mod
    except Exception:
        return
    BufCls = getattr(_buf_mod, 'AFCTrigger', None)
    if BufCls is None or getattr(BufCls, '_afc_steppermless_patched', False):
        return
    _timeout = getattr(_buf_mod, 'CHECK_RUNOUT_TIMEOUT', 0.5)
    _orig = BufCls.extruder_pos_update_event

    def _guarded(self, eventtime):
        cur = self.current_lane
        if cur is not None and getattr(cur, 'extruder_stepper', None) is None:
            return eventtime + _timeout  # no stepper to track; skip fault check
        return _orig(self, eventtime)

    BufCls.extruder_pos_update_event = _guarded
    BufCls._afc_steppermless_patched = True


def _patch_afc_hub_virtual_state():
    """3. AFC_hub: a driven virtual hub reports _state (hub HES), not F1S."""
    try:
        from extras import AFC_hub as _hub_mod
    except Exception:
        return
    HubCls = getattr(_hub_mod, 'afc_hub', None)
    if HubCls is None or getattr(HubCls, '_afc_virtual_state_patched', False):
        return
    _orig_prop = HubCls.state
    _orig_cb = HubCls.switch_pin_callback

    def _state_getter(self):
        if self.is_virtual_pin() and getattr(self, '_state_driven', False):
            return bool(self._state)
        return _orig_prop.fget(self)

    def _cb(self, eventtime, state):
        self._state_driven = True
        return _orig_cb(self, eventtime, state)

    HubCls.state = property(_state_getter)
    HubCls.switch_pin_callback = _cb
    HubCls._afc_virtual_state_patched = True


def _patch_afc_unload_shared_phase():
    """4. AFC.load_sequence / unload_sequence: run the shared toolhead ops that
    upstream's custom_load_cmd / custom_unload_cmd branches skip.

    Upstream treats a custom command as a complete replacement: the unload
    branch skips heat/cut/park/form_tip (+post_unload_macro), and the load
    branch skips _check_extruder_temp. Our serial units (ACE/OpenAMS) set those
    commands to just the hardware transport and rely on the shared phase (as our
    two-phase load/unload did) — otherwise the transport / post-load purge hits
    a cold extruder and park/form_tip/exit-bin never fire."""
    try:
        from extras import AFC as _afc_mod
    except Exception:
        return
    AFCcls = getattr(_afc_mod, 'afc', None)
    if AFCcls is None or getattr(AFCcls, '_afc_custom_unload_phase_patched', False):
        return
    _orig_unload = AFCcls.unload_sequence
    _orig_load = AFCcls.load_sequence

    def _afc_shared_toolhead_unload(self, cur_lane, cur_extruder):
        # lane_unloading: LED + (ACE/OpenAMS) follower/assist stop.
        cur_lane.unit_obj.lane_unloading(cur_lane)
        if self._check_extruder_temp(cur_lane):
            self.afcDeltaTime.log_with_time("Done heating toolhead")
        self.move_e_pos(-2, cur_extruder.tool_unload_speed, "Quick Pull",
                        wait_tool=False)
        cur_lane.disable_buffer()
        cur_lane.sync_to_extruder()
        cur_lane.select_lane()
        if self.tool_cut:
            cur_lane.extruder_obj.estats.increase_cut_total()
            self.gcode.run_script_from_command(
                "{} EXTRUDER={}".format(self.tool_cut_cmd, cur_extruder.name))
            if self.park:
                self.gcode.run_script_from_command(
                    "{} EXTRUDER={}".format(self.park_cmd, cur_extruder.name))
        if self.form_tip:
            if self.park:
                self.gcode.run_script_from_command(
                    "{} EXTRUDER={}".format(self.park_cmd, cur_extruder.name))
            if self.form_tip_cmd == "AFC":
                self.printer.lookup_object('AFC_form_tip').tip_form()
            else:
                self.gcode.run_script_from_command(self.form_tip_cmd)

    def _wrapped_unload(self, cur_lane, cur_hub, cur_extruder):
        is_custom = bool(cur_lane.custom_unload_cmd)
        if is_custom:
            try:
                self._afc_shared_toolhead_unload(cur_lane, cur_extruder)
            except Exception as e:
                self.logger.error(
                    "AFC shared toolhead unload phase error: %s" % e)
        result = _orig_unload(self, cur_lane, cur_hub, cur_extruder)
        # Upstream runs post_unload_macro only in the stepper branch, so a custom
        # unload skips it (e.g. SNAPMAKER_EXIT_DISCARD_BIN). Run it here, after
        # the transport. (The load side already runs post_load_macro for both.)
        if is_custom and self.post_unload_macro is not None:
            try:
                self.gcode.run_script_from_command(self.post_unload_macro)
            except Exception as e:
                self.logger.error("AFC post_unload_macro error: %s" % e)
        return result

    def _wrapped_load(self, cur_lane, cur_hub, cur_extruder):
        # Upstream's custom_load_cmd branch skips _check_extruder_temp (only the
        # stepper branch heats), so the post-load poop/purge runs cold. Heat
        # (and wait) here before the custom transport.
        if cur_lane.custom_load_cmd:
            try:
                if self._check_extruder_temp(cur_lane):
                    self.afcDeltaTime.log_with_time("Done heating toolhead")
            except Exception as e:
                self.logger.error("AFC custom-load heat error: %s" % e)
        return _orig_load(self, cur_lane, cur_hub, cur_extruder)

    AFCcls._afc_shared_toolhead_unload = _afc_shared_toolhead_unload
    AFCcls.unload_sequence = _wrapped_unload
    AFCcls.load_sequence = _wrapped_load
    AFCcls._afc_custom_unload_phase_patched = True


def _patch_afc_bowden_serial_unit():
    """6. afcFunction.cmd_CALIBRATE_AFC: let steppermless serial units (ACE,
    OpenAMS) run their self-contained bowden / PTFE-length calibration.

    Upstream's BOWDEN= branch insists on a tool_start sensor (or a tool_end +
    turtleneck-buffer pair) and otherwise aborts with:

        "Cannot calibrate with only post extruder sensor and no turtleneck
         buffer defined in config"

    Our serial lanes have no extruder_stepper and no tool_start — their
    calibrate_bowden is driven by the unit firmware (e.g. ACE feed-to-toolhead-
    sensor, OpenAMS OAMS_CALIBRATE_PTFE_LENGTH), so the toolhead-sensor guard
    doesn't apply to them. For such a BOWDEN lane, temporarily satisfy the guard
    (a tool_start sentinel) so upstream proceeds straight to
    unit.calibrate_bowden(), then restore tool_start. This reproduces the
    custom-calibration bypass our deviated fork carried, without editing
    upstream. The guard value is never consumed past the check on this path, and
    our serial calibrate_bowden implementations ignore tool_start entirely."""
    try:
        from extras import AFC_functions as _fn_mod
    except Exception:
        return
    FnCls = getattr(_fn_mod, 'afcFunction', None)
    if FnCls is None or getattr(FnCls, '_afc_bowden_serial_patched', False):
        return
    _orig = FnCls.cmd_CALIBRATE_AFC
    _SENTINEL = "__afc_compat_serial_cal__"

    def _wrapped(self, gcmd):
        afc_bl = gcmd.get('BOWDEN', None)
        cur_lane = self.afc.lanes.get(afc_bl) if afc_bl is not None else None
        # Only step in for a steppermless serial lane whose extruder has no
        # tool_start sensor (the exact case upstream wrongly rejects).
        bypass = (cur_lane is not None
                  and getattr(cur_lane, 'extruder_stepper', None) is None
                  and getattr(cur_lane.extruder_obj, 'tool_start', None) is None)
        if not bypass:
            return _orig(self, gcmd)
        ext = cur_lane.extruder_obj
        ext.tool_start = _SENTINEL  # skip upstream's tool_start guard block
        try:
            return _orig(self, gcmd)
        finally:
            # Upstream never touches tool_start on this path; restore our None.
            if ext.tool_start == _SENTINEL:
                ext.tool_start = None

    FnCls.cmd_CALIBRATE_AFC = _wrapped
    FnCls._afc_bowden_serial_patched = True


def _patch_afc_lane_load_runout():
    """7. AFCLane.handle_load_runout: generalize the load/runout callback so the
    serial-polled units (ACE, OpenAMS) can drive it without crashing.

    Upstream's version is HTLF-only and assumes a physical load switch:
      * it dereferences self.load_debounce_button (and, in the runout branch,
        self.fila_load) which only exist when the lane has a `load:` pin. Our
        serial lanes have neither, so polling F1S through handle_load_runout
        AttributeErrors and shuts Klipper down the moment a spool is removed;
      * it gates the load/insert/runout body on type == "HTLF", so AMS/ACE never
        run their own insert/runout handling.

    Reproduce our deviated fork's generalized callback: guard the optional
    physical-switch objects, extend the gate to the serial unit types (and any
    unit exposing the on_filament_* hooks), and route a runout through the unit's
    own handler when it provides one (e.g. OAMS can't unload once F1S is empty).
    Falls back to upstream's infinite-spool / pause behaviour otherwise. HTLF
    lanes keep behaving exactly as before (button + fila_load present, hooks
    guarded by hasattr)."""
    try:
        from extras import AFC_lane as _lane_mod
    except Exception:
        return
    LaneCls = getattr(_lane_mod, 'AFCLane', None)
    if LaneCls is None or getattr(LaneCls, '_afc_load_runout_patched', False):
        return

    # Units whose prep and load are the same (serial-polled) sensor.
    _ONLY_LOAD_TYPES = ("HTLF", "Claymore", "OpenAMS", "ACE")

    def handle_load_runout(self, eventtime, load_state):
        # Register state with the physical filament switch if this lane has one;
        # serial-polled AMS/ACE lanes have no load: pin, so no debounce button.
        button = getattr(self, 'load_debounce_button', None)
        if button is not None:
            try:
                button._old_note_filament_present(is_filament_present=load_state)
            except Exception:
                button._old_note_filament_present(eventtime, load_state)

        unit = self.unit_obj
        is_only_load = (unit.type in _ONLY_LOAD_TYPES
                        or hasattr(unit, 'on_filament_remove'))
        if (self.printer.state_message == 'Printer is ready'
                and is_only_load
                and self._afc_prep_done is True):
            if load_state:
                # Stash any externally-staged spool (scanner -> next_spool_id)
                # before set_loaded() consumes it via _set_values, so a unit's
                # on_filament_insert (e.g. ACE, which clear_values()) can tell a
                # fresh scan apart from a stale/remembered id and keep it.
                try:
                    self._afc_staged_spool_id = getattr(
                        self.afc.spool, 'next_spool_id', None)
                except Exception:
                    self._afc_staged_spool_id = None
                self.set_loaded()
                # on_filament_insert only when this wasn't a suppressed
                # (operation-driven) state change.
                if not getattr(self, '_load_suppressed', False):
                    if hasattr(unit, 'on_filament_insert'):
                        unit.on_filament_insert(self)
                self._load_suppressed = False

                if self.td1_device_id and not self.tool_loaded:
                    self._prep_capture_td1()

                if self.hub == 'direct_load':  # TODO: is_direct_hub
                    if self.afc.function.is_printing(check_movement=True):
                        self.afc.error.AFC_error(
                            "Cannot load spool to toolhead while printer is "
                            "actively moving or homing", False)
                    else:
                        self.afc.TOOL_LOAD(self)

                self._post_prep_user_macro()
            else:
                if hasattr(unit, 'on_filament_remove'):
                    unit.on_filament_remove(self)
                # Don't run if user disabled the sensor in the GUI.
                fila_load = getattr(self, 'fila_load', None)
                if (fila_load is not None
                        and not fila_load.runout_helper.sensor_enabled
                        and self.afc.function.is_printing()):
                    self.logger.warning(
                        "Load runout has been detected, but pause and runout "
                        "detection has been disabled")
                elif unit.check_runout(self):
                    # Let the unit handle runout if it provides custom logic
                    # (e.g. OAMS cannot unload once F1S is empty).
                    handler = getattr(unit, 'handle_runout', None)
                    if handler is not None and handler(self):
                        pass
                    elif self.runout_lane is not None:
                        self._perform_infinite_runout()
                    else:
                        self._perform_pause_runout()
                elif self.status != "calibrating":
                    self.set_unloaded()

        self.afc.save_vars()

    LaneCls.handle_load_runout = handle_load_runout
    LaneCls._afc_load_runout_patched = True


def _patch_afc_unit_filament_hooks():
    """8. afcUnit.on_filament_insert / on_filament_remove: provide the base
    methods our serial units' overrides call via super().

    Our ACE / OpenAMS on_filament_insert() end with super().on_filament_insert(),
    expecting the base unit to fire the ``afc:lane_inserted`` event. Upstream's
    afcUnit has no such method, so the super() call AttributeErrors when a spool
    is inserted into an empty lane. Add the base hooks (insert -> send event,
    remove -> no-op) when upstream lacks them."""
    try:
        from extras import AFC_unit as _unit_mod
    except Exception:
        return
    UnitCls = getattr(_unit_mod, 'afcUnit', None)
    if UnitCls is None or getattr(UnitCls, '_afc_filament_hooks_patched', False):
        return

    if not hasattr(UnitCls, 'on_filament_insert'):
        def on_filament_insert(self, lane):
            # Fired after set_loaded() when filament is newly detected; the U1
            # bridge listens for this. Subclasses override for RFID sync etc.
            self.printer.send_event("afc:lane_inserted", lane)
        UnitCls.on_filament_insert = on_filament_insert

    if not hasattr(UnitCls, 'on_filament_remove'):
        def on_filament_remove(self, lane):
            # Base no-op; subclasses override for inventory cleanup.
            pass
        UnitCls.on_filament_remove = on_filament_remove

    UnitCls._afc_filament_hooks_patched = True


def _patch_afc_hub_virtual_load_check():
    """9. afc_hub.handle_connect: a virtual hub only needs a lane's load sensor
    when that lane also has a prep sensor.

    Upstream rejects ANY lane without a `load:` pin on a `switch_pin: virtual`
    hub ("...need load sensors for virtual hub sensor to work correctly"). Our
    serial lanes (OpenAMS/ACE) are fully sensorless — no load AND no prep, their
    load state is driven by the unit poller — so they're valid on a virtual hub.
    Relax the check to error only when load is None AND prep is not None (a lane
    that has a prep switch but no load switch genuinely can't drive the virtual
    hub). This also makes the check order-independent: upstream only passed when
    a hub's handle_connect happened to run before its lanes registered."""
    try:
        from extras import AFC_hub as _hub_mod
    except Exception:
        return
    HubCls = getattr(_hub_mod, 'afc_hub', None)
    if HubCls is None or getattr(HubCls, '_afc_virtual_load_check_patched', False):
        return
    _config_error = getattr(_hub_mod, 'config_error', None)

    def handle_connect(self):
        self.gcode = self.afc.gcode
        self.reactor = self.afc.reactor
        self.printer.send_event("afc_hub:register_macros", self)
        if self.is_virtual_pin():
            msg = ("The following lanes need load sensors for virtual hub "
                   "sensor to work correctly:")
            report_error = False
            for lane in self.lanes.values():
                # Only a lane that HAS a prep switch but NO load switch breaks a
                # virtual hub; fully sensorless serial lanes are fine.
                if lane.load is None and lane.prep is not None:
                    report_error = True
                    msg += "\n%s" % lane.fullname
            if report_error:
                err = _config_error or self.printer.config_error
                raise err(msg)

    HubCls.handle_connect = handle_connect
    HubCls._afc_virtual_load_check_patched = True


def apply_compat_patches():
    """Apply all AFC compatibility shims. Idempotent; safe to call repeatedly
    and from multiple unit modules."""
    _patch_afc_lane_virtual_hub()
    _patch_afc_buffer_steppermless()
    _patch_afc_hub_virtual_state()
    _patch_afc_unload_shared_phase()
    _patch_afc_bowden_serial_unit()
    _patch_afc_lane_load_runout()
    _patch_afc_unit_filament_hooks()
    _patch_afc_hub_virtual_load_check()
