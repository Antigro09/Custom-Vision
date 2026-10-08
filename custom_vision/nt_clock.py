"""Version-scoped conversion from host read-completion to the NT server clock.

The installed producer dependency is pyntcore >=2023.4,<2025. For that API,
``ntcore._now()`` and ``getServerTimeOffset()`` are integer microseconds. The
private ``_now`` binding is isolated here so an API change fails closed rather
than silently interpreting nanoseconds as microseconds. This adapter does not
claim exposure timing, hardware clock alignment, or measured capture correction.

Upstream references checked for this implementation (the installed 2024.3.2.1
native ``_now.__doc__`` also confirms integer 1-us increments):
https://pypi.org/project/pyntcore/2024.3.2.1/
https://github.com/robotpy/pyntcore/blob/2023.4.3.0/gen/ntcore_cpp.yml
https://github.com/robotpy/pyntcore/blob/2023.4.3.0/ntcore/__init__.py
https://github.com/wpilibsuite/allwpilib/blob/v2024.3.2/ntcore/src/main/native/include/ntcore_cpp.h
"""

from decimal import Decimal, InvalidOperation
from importlib import metadata
import re
import time


MAX_TIMESTAMP_US = (1 << 63) - 1
MIN_OFFSET_US = -(1 << 63)


def _integer(value, minimum, maximum):
    # bool is an int subclass but cannot be a timestamp or clock offset.
    return type(value) is int and minimum <= value <= maximum


def _microsecond_api(version):
    """Accept released versions in the dependency's verified API family only."""
    if not isinstance(version, str):
        return False
    match = re.fullmatch(r"(2023|2024)\.(\d+)(?:\.\d+)+(?:\.post\d+)?", version)
    return bool(match and (int(match[1]) == 2024 or int(match[2]) >= 4))


def _correction_us(correction_ms):
    """Quantize a configured nonnegative correction toward zero in whole us.

    Decimal conversion avoids a float multiply changing the whole-us boundary.
    The configured millisecond number is not itself a measurement: zero and
    nonzero corrections both require separate verification metadata.
    """
    if type(correction_ms) not in (int, float):
        return None
    try:
        value = Decimal(str(correction_ms))
        if not value.is_finite() or value < 0:
            return None
        microseconds = value * 1000
        if microseconds > MAX_TIMESTAMP_US:
            return None
        return int(microseconds)
    except (InvalidOperation, ValueError, OverflowError):
        return None


class NtClockAdapter:
    """Fail-closed, exact-integer microsecond clock adapter for the producer.

    ``ntcore_module`` and ``package_version`` may be supplied by a test or an
    explicitly pinned harness. Production discovers the installed distribution;
    the only supported unit profile is the released 2023.4/2024 microsecond API.
    ``monotonic_ns`` is injectable; default clock/API lookups remain dynamic so
    callers can use existing monkeypatch paths without importing NTCore offline.
    """

    def __init__(self, ntcore_module=None, package_version=None, monotonic_ns=None):
        self._ntcore = ntcore_module
        self._monotonic_ns = monotonic_ns
        if package_version is None:
            # Distribution metadata is authoritative even when __version__ is
            # absent (or the development checkout reports "master").
            try:
                package_version = metadata.version("pyntcore")
            except metadata.PackageNotFoundError:
                package_version = getattr(ntcore_module, "__version__", None)
        self.package_version = package_version
        self.supported = _microsecond_api(package_version)

    def capture_server_us(self, instance, capture_monotonic_us, correction_ms=0):
        """Return an integer server timestamp, or None when sync is unusable.

        The caller separately gates camera validity; ``instance.isConnected()``
        is the NT connection state. A signed offset is valid and can be negative.
        Capture timestamps, clocks and the computed result must be positive
        signed-64-bit microseconds. Timestamp zero is reserved for typed invalid
        output. A capture in the future is invalid, not clamped to a recent time.
        No cached offset is reused after disconnect; exceptions suppress sync.
        """
        if not self.supported or instance is None:
            return None
        if not _integer(capture_monotonic_us, 1, MAX_TIMESTAMP_US):
            return None
        correction_us = _correction_us(correction_ms)
        if correction_us is None:
            return None
        try:
            if instance.isConnected() is not True:
                return None
            offset_us = instance.getServerTimeOffset()
            if not _integer(offset_us, MIN_OFFSET_US, MAX_TIMESTAMP_US):
                return None
            ntcore_module = self._ntcore
            if ntcore_module is None:
                import ntcore as ntcore_module
            nt_now_us = ntcore_module._now()
            monotonic_ns = (self._monotonic_ns or time.monotonic_ns)()
            # Test the ns value before integer division; floats/bools must not
            # become plausible-looking integer microseconds through coercion.
            if type(monotonic_ns) is not int or monotonic_ns <= 0:
                return None
            monotonic_us = monotonic_ns // 1000
            if not _integer(nt_now_us, 1, MAX_TIMESTAMP_US):
                return None
            if not _integer(monotonic_us, 1, MAX_TIMESTAMP_US):
                return None
            age_us = monotonic_us - capture_monotonic_us
            if age_us < 0:
                return None
            capture_server_us = nt_now_us + offset_us - age_us - correction_us
            if not _integer(capture_server_us, 1, MAX_TIMESTAMP_US):
                return None
            # An offset may still be returned during a disconnect transition.
            if instance.isConnected() is not True:
                return None
            return capture_server_us
        except Exception:
            # A missing private binding, native failure or clock failure is not
            # evidence that a synchronized timestamp can be supplied.
            return None
