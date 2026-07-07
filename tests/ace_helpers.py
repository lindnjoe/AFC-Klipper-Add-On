"""
Typed test fakes for the ACE/ACE2 unit tests.

These replace broad MagicMock usage: every fake declares its real attribute
set explicitly, so a typo'd or missing attribute fails the test instead of
silently returning a truthy MagicMock. Callables under observation are
Recorder instances (plain callables that record their calls).

Self-contained on purpose: the ACE tests must not import from the OpenAMS
test helpers so the ACE work can be upstreamed on its own.
"""

from __future__ import annotations


class Recorder:
    """A plain callable that records calls; optionally returns a fixed value
    or raises. Replaces MagicMock for callbacks/methods under observation."""

    def __init__(self, result=None, raises=None):
        self.calls = []
        self.result = result
        self.raises = raises

    def __call__(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        if self.raises is not None:
            raise self.raises
        return self.result

    @property
    def called(self):
        return len(self.calls) > 0

    @property
    def call_count(self):
        return len(self.calls)

    @property
    def last_args(self):
        return self.calls[-1][0]

    @property
    def last_kwargs(self):
        return self.calls[-1][1]


class FakeLogger:
    """Records log lines per level."""

    def __init__(self):
        self.lines = {"info": [], "debug": [], "warning": [], "error": []}

    def info(self, msg, *a, **k):
        self.lines["info"].append(msg)

    def debug(self, msg, *a, **k):
        self.lines["debug"].append(msg)

    def warning(self, msg, *a, **k):
        self.lines["warning"].append(msg)

    def error(self, msg, *a, **k):
        self.lines["error"].append(msg)


class FakeReactor:
    NOW = 0.0
    NEVER = 9_999_999_999.0

    def __init__(self, monotonic_value=100.0):
        self._monotonic = monotonic_value
        self.register_callback = Recorder()
        self.register_timer = Recorder(result="timer-handle")
        self.unregister_timer = Recorder()
        self.update_timer = Recorder()

    def monotonic(self):
        return self._monotonic

    def pause(self, until):
        pass

    def completion(self):
        comp = Recorder()
        comp.complete = Recorder()
        return comp


class FakeError:
    """Stands in for afc.error: records AFC_error calls."""

    def __init__(self):
        self.AFC_error = Recorder()


class FakeFunction:
    """Stands in for afc.function with explicit printing/pause state."""

    def __init__(self, printing=False, paused=False, in_print_flag=False):
        self.printing = printing
        self.paused = paused
        self.in_print_flag = in_print_flag
        self.raise_on_is_printing = None

    def is_printing(self, check_movement=False):
        if self.raise_on_is_printing is not None:
            raise self.raise_on_is_printing
        return self.printing

    def is_paused(self):
        return self.paused

    def in_print(self):
        return self.in_print_flag


class FakeAFC:
    """Explicit-attribute afc object for unit tests."""

    def __init__(self):
        self.lanes = {}
        self.current = None
        self.in_toolchange = False
        self.error = FakeError()
        self.function = FakeFunction()
        self.reactor = FakeReactor()
        self.logger = FakeLogger()
        self.save_vars = Recorder()
        self.load_to_hub = False


class FakeExtruderObj:
    """AFC_extruder stand-in: section name, physical name, loaded lane."""

    def __init__(self, name="extruder", th_extruder_name=None, lane_loaded=None):
        self.name = name
        self.th_extruder_name = th_extruder_name if th_extruder_name is not None else name
        self.lane_loaded = lane_loaded


class FakeLane:
    """AFCLane stand-in with the explicit state the ACE/OpenAMS units touch."""

    def __init__(self, name, extruder_obj=None, hub_obj=None,
                 tool_loaded=False, runout_lane=None, status=None):
        self.name = name
        self.extruder_obj = extruder_obj
        self.hub_obj = hub_obj
        self.tool_loaded = tool_loaded
        self.runout_lane = runout_lane
        self.status = status
        self.prep_state = False
        self.loaded_to_hub = False
        self._load_state = False
        self._load_suppressed = False
        self._afc_prep_done = True
        self._oams_runout_detected = False
        self._oams_runout_empty = False
        self.load_to_hub = False
        self.use_feed_assist = None
        # Observable lane methods
        self.handle_load_runout = Recorder()
        self.sync_to_extruder = Recorder()
        self.unsync_to_extruder = Recorder()
        self.enable_buffer = Recorder()
        self.set_tool_unloaded = Recorder()
        self.get_toolhead_pre_sensor_state = Recorder(result=False)


class FakeHub:
    """AFC_hub stand-in; virtual=None means 'real switch hub' (no
    is_virtual_pin attribute at all)."""

    def __init__(self, virtual=True):
        self._virtual = virtual

    def is_virtual_pin(self):
        return self._virtual


class FakeToolheadPrinter:
    """printer stand-in for _active_assist_lane: lookup_object('toolhead')
    resolves to an object whose get_extruder().get_name() is the active
    extruder. Set active_extruder=None to make the lookup raise."""

    class _Extruder:
        def __init__(self, name):
            self._name = name

        def get_name(self):
            return self._name

    class _Toolhead:
        def __init__(self, name):
            self._extruder = FakeToolheadPrinter._Extruder(name)

        def get_extruder(self):
            return self._extruder

    def __init__(self, active_extruder="extruder"):
        self.active_extruder = active_extruder
        self.state_message = "Printer is ready"

    def lookup_object(self, name, default=None):
        if name == "toolhead":
            if self.active_extruder is None:
                raise RuntimeError("no toolhead")
            return self._Toolhead(self.active_extruder)
        return default


class FakeGcmd:
    """Gcode command stand-in: params dict + response capture; error() returns
    an exception instance (the code does `raise gcmd.error(...)`)."""

    def __init__(self, **params):
        self._params = params
        self.responses = []

    def get_int(self, name, default=None):
        val = self._params.get(name, default)
        return int(val) if val is not None else None

    def get(self, name, default=None):
        return self._params.get(name, default)

    def respond_info(self, msg):
        self.responses.append(msg)

    def error(self, msg):
        return RuntimeError(msg)


class FakeAce:
    """ACE serial connection stand-in: records the commands the unit sends."""

    def __init__(self, connected=True):
        self.connected = connected
        self.start_feed_assist = Recorder()
        self.stop_feed_assist = Recorder()
        self.send_command_async = Recorder()
