# Construction tests for extras/AFC_ACE2.py.
#
# The ACE 2 Pro inherits nearly everything from the V1 ACE and overrides a
# handful of values that are wrong for the newer hardware. Those overrides are
# the whole point of the subclass, and each one is a field failure if it
# regresses:
#
#   * 230400 baud. At the V1's 115200 the unit never sees a valid frame and
#     never replies -- it looks dead rather than misconfigured.
#   * the encoder feed-check window, where an unreachable threshold makes EVERY
#     feed raise FEED_ERROR.
#   * stuck-spool detection defaulting ON, because unlike the V1 the ACE2 has a
#     real encoder and reports a true mechanical-jam state.
#
# The parent's __init__ needs the whole Klipper/AFC stack, so it is stubbed:
# what is under test here is the subclass's own configuration.
from __future__ import annotations

import types

import pytest

import extras.AFC_ACE2 as ace2mod
from extras.AFC_ACE2 import afcACE2, ACE2_ENCODER_SCALE


class _ConfigError(Exception):
    pass


class _Config:
    def __init__(self, **opts):
        self._o = opts
        self.error = _ConfigError

    def get_name(self):
        return "AFC_ACE2 Ace2_1"

    def get(self, key, default=None):
        return self._o.get(key, default)

    def getint(self, key, default=None, **kw):
        v = self._o.get(key, default)
        return int(v) if v is not None else None

    def getfloat(self, key, default=None, **kw):
        v = self._o.get(key, default)
        return float(v) if v is not None else None

    def getboolean(self, key, default=None, **kw):
        v = self._o.get(key, default)
        return default if v is None else bool(v)


@pytest.fixture(autouse=True)
def _stub_parent(monkeypatch):
    """Neutralise afcACE.__init__ -- the parent needs a printer, a serial port
    and lanes; the subclass's own configuration does not."""
    monkeypatch.setattr(ace2mod.afcACE, "__init__",
                        lambda self, config: None)


def _make(**opts):
    obj = afcACE2.__new__(afcACE2)
    afcACE2.__init__(obj, _Config(**opts))
    return obj


class TestDefaults:
    def test_type_defaults_to_ace2(self):
        assert _make().type == "ACE2"

    def test_type_can_be_overridden(self):
        assert _make(type="ACE2_custom").type == "ACE2_custom"

    def test_baud_defaults_to_230400_not_the_v1_115200(self):
        # The single most consequential override: at 115200 the ACE2 never
        # sees a valid frame and never answers, which reads as dead hardware.
        assert _make().baud_rate == 230400

    def test_baud_can_be_overridden(self):
        assert _make(baud_rate=115200).baud_rate == 115200

    def test_dryer_ceiling_is_70_not_the_v1_55(self):
        assert _make().max_dryer_temperature == 70.0

    def test_dryer_ceiling_can_be_overridden(self):
        assert _make(max_dryer_temperature=60).max_dryer_temperature == 60.0

    def test_stuck_detection_defaults_on_for_the_encoder_equipped_ace2(self):
        # The parent reads the same key defaulting False; the ACE2 has a real
        # encoder reporting a true jam state, so it defaults True here.
        assert _make()._stuck_detection is True

    def test_stuck_detection_can_be_disabled(self):
        assert _make(stuck_spool_detection=False)._stuck_detection is False

    def test_feed_check_window_defaults(self):
        obj = _make()
        assert (obj.feed_check_length, obj.feed_error_length) == (200, 185)


class TestFeedCheckValidation:
    """The encoder can only ever reach feed_error_length * 1.2342. A
    feed_check_length at or above that is unreachable, so EVERY feed would
    raise FEED_ERROR -- a misconfiguration that presents as broken hardware."""

    def test_the_default_pair_is_valid(self):
        _make()                       # must not raise

    def test_an_unreachable_threshold_is_rejected(self):
        # 185 * 1.2342 = 228.3; asking for 229 can never be satisfied.
        with pytest.raises(_ConfigError) as e:
            _make(feed_check_length=229, feed_error_length=185)
        assert "can never reach it" in str(e.value)

    def test_exactly_at_the_ceiling_is_rejected(self):
        err = int(100)
        unreachable = int(err * ACE2_ENCODER_SCALE)      # 123
        with pytest.raises(_ConfigError):
            _make(feed_check_length=unreachable + 1, feed_error_length=err)

    def test_just_under_the_ceiling_is_accepted(self):
        obj = _make(feed_check_length=120, feed_error_length=100)
        assert obj.feed_check_length == 120

    def test_the_message_names_the_section_and_both_numbers(self):
        with pytest.raises(_ConfigError) as e:
            _make(feed_check_length=250, feed_error_length=185)
        msg = str(e.value)
        assert "AFC_ACE2 Ace2_1" in msg and "250" in msg
        assert "tolerance_mm" in msg          # tells the operator what to change

    def test_lowering_feed_check_widens_tolerance_without_moving_the_check(self):
        # The documented tuning knob: same checkpoint, larger slip allowance.
        wide = _make(feed_check_length=100, feed_error_length=185)
        assert wide.feed_error_length == 185 and wide.feed_check_length == 100


class TestLoader:
    def test_load_config_prefix_builds_the_unit(self):
        obj = ace2mod.load_config_prefix(_Config())
        assert isinstance(obj, afcACE2)
        assert obj.baud_rate == 230400
