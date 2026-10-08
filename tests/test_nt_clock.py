"""Clock conversion tests require no native NTCore or camera."""
from types import SimpleNamespace

import pytest

from custom_vision.nt_clock import MAX_TIMESTAMP_US, NtClockAdapter


class Instance:
    def __init__(self, offset=2_000_000, connected=True):
        self.offset = offset
        self.connected = connected

    def isConnected(self):
        return self.connected

    def getServerTimeOffset(self):
        return self.offset


def clock(version="2024.3.2.1", nt_now=500_000, monotonic_ns=1_000_000_000):
    return NtClockAdapter(
        SimpleNamespace(_now=lambda: nt_now), version, lambda: monotonic_ns
    )


def test_server_timestamp_uses_age_and_configured_correction_in_integer_us():
    timestamp = clock().capture_server_us(Instance(), 970_000, 4)
    assert type(timestamp) is int
    assert timestamp == 2_466_000


@pytest.mark.parametrize("version", ["2023.4.0.0", "2023.4.3.0", "2024.1.1.0", "2024.3.2.1"])
def test_verified_microsecond_version_family(version):
    assert clock(version).capture_server_us(Instance(), 970_000) == 2_470_000


@pytest.mark.parametrize("version", [None, "", "master", "2023.3.0.0", "2025.1.1.0", "2026.2.2", "2027.0.0-alpha-7", "2024.3.2.dev1"])
def test_unsupported_or_unknown_api_does_not_guess_units(version, monkeypatch):
    monkeypatch.setattr("custom_vision.nt_clock.metadata.version", lambda _: version)
    assert clock(version).capture_server_us(Instance(), 970_000) is None


def test_distribution_version_discovery(monkeypatch):
    monkeypatch.setattr("custom_vision.nt_clock.metadata.version", lambda _: "2024.3.2.1")
    adapter = NtClockAdapter(SimpleNamespace(_now=lambda: 500_000), monotonic_ns=lambda: 1_000_000_000)
    assert adapter.package_version == "2024.3.2.1"
    assert adapter.capture_server_us(Instance(), 970_000) == 2_470_000


def test_no_native_dependency_is_needed_when_publisher_is_disabled():
    assert clock().capture_server_us(None, 970_000) is None


@pytest.mark.parametrize("offset", [None, True, False, "2000000", 2_000_000.0, float("nan"), float("inf"), -(1 << 63)-1, 1 << 63])
def test_missing_or_invalid_offset_suppresses_sync(offset):
    assert clock().capture_server_us(Instance(offset), 970_000) is None


def test_negative_offset_is_supported_when_result_is_usable():
    assert clock(nt_now=3_000_000).capture_server_us(Instance(-500_000), 970_000) == 2_470_000


@pytest.mark.parametrize("value", [None, True, False, "970000", 970_000.0, float("nan"), float("inf"), -1, 0, 1 << 63, 1_000_001])
def test_missing_invalid_or_future_capture_suppresses_sync(value):
    assert clock().capture_server_us(Instance(), value) is None


@pytest.mark.parametrize("value", [None, True, "500000", 500_000.0, float("nan"), float("inf"), -1, 0, 1 << 63])
def test_unusable_nt_clock_suppresses_sync(value):
    assert clock(nt_now=value).capture_server_us(Instance(), 970_000) is None


@pytest.mark.parametrize("value", [None, True, "1000000000", 1_000_000_000.0, float("nan"), float("inf"), -1, 0, 999, (1 << 63)*1000])
def test_unusable_monotonic_clock_suppresses_sync(value):
    assert clock(monotonic_ns=value).capture_server_us(Instance(), 970_000) is None


@pytest.mark.parametrize("correction", [None, True, "4", -1, float("nan"), float("inf"), 1 << 63])
def test_unusable_correction_suppresses_sync(correction):
    assert clock().capture_server_us(Instance(), 970_000, correction) is None


def test_zero_correction_and_sub_microsecond_quantization():
    # The verified flag is intentionally external. A measured zero produces
    # exactly the same clock math as configured zero with unknown verification.
    assert clock().capture_server_us(Instance(), 970_000, 0) == 2_470_000
    assert clock().capture_server_us(Instance(), 970_000, 0.0009) == 2_470_000
    assert clock().capture_server_us(Instance(), 970_000, 0.001) == 2_469_999


def test_integer_timestamps_above_float_exactness_limit_are_preserved():
    start = (1 << 53) + 7
    adapter = clock(nt_now=start, monotonic_ns=(start + 30_000)*1000)
    timestamp = adapter.capture_server_us(Instance(2_000_000), start, 4)
    assert type(timestamp) is int
    assert timestamp == start + 1_966_000


@pytest.mark.parametrize("nt_now,offset,capture", [(1, 0, 1), (MAX_TIMESTAMP_US, 3, 1)])
def test_zero_negative_or_overflow_result_is_suppressed(nt_now, offset, capture):
    assert clock(nt_now=nt_now, monotonic_ns=1000).capture_server_us(Instance(offset), capture, 0.002) is None


def test_disconnect_suppresses_even_a_stale_offset_and_reconnect_requires_new_offset():
    instance = Instance(connected=False)
    adapter = clock()
    assert adapter.capture_server_us(instance, 970_000) is None
    instance.connected = True
    instance.offset = None
    assert adapter.capture_server_us(instance, 970_000) is None
    instance.offset = 2_000_000
    assert adapter.capture_server_us(instance, 970_000) == 2_470_000


def test_disconnect_during_sampling_suppresses_sync():
    instance = Instance()
    def now():
        instance.connected = False
        return 500_000
    adapter = NtClockAdapter(SimpleNamespace(_now=now), "2024.3.2.1", lambda: 1_000_000_000)
    assert adapter.capture_server_us(instance, 970_000) is None


@pytest.mark.parametrize("method", ["isConnected", "getServerTimeOffset", "_now", "monotonic_ns"])
def test_clock_api_exceptions_suppress_sync(method):
    def fail():
        raise RuntimeError("unavailable")
    instance = Instance()
    ntcore = SimpleNamespace(_now=lambda: 500_000)
    monotonic_ns = lambda: 1_000_000_000
    if method in ("isConnected", "getServerTimeOffset"):
        setattr(instance, method, fail)
    elif method == "_now":
        ntcore._now = fail
    else:
        monotonic_ns = fail
    assert NtClockAdapter(ntcore, "2024.3.2.1", monotonic_ns).capture_server_us(instance, 970_000) is None


def test_missing_private_binding_suppresses_sync():
    adapter = NtClockAdapter(SimpleNamespace(), "2024.3.2.1", lambda: 1_000_000_000)
    assert adapter.capture_server_us(Instance(), 970_000) is None


def test_default_clock_lookups_remain_dynamic(monkeypatch):
    ntcore = SimpleNamespace(_now=lambda: 1)
    adapter = NtClockAdapter(ntcore, "2024.3.2.1")
    ntcore._now = lambda: 500_000
    monkeypatch.setattr("custom_vision.nt_clock.time.monotonic_ns", lambda: 1_000_000_000)
    assert adapter.capture_server_us(Instance(), 970_000) == 2_470_000
