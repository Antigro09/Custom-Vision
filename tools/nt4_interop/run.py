#!/usr/bin/env python3
"""Bounded localhost test of the actual Publisher against released alpha-7 NTCore.

Run from an isolated Python 3.12+ environment with requirements.txt installed.
The fixture is a producer golden JSON packet, never a malformed consumer case.
This is a desktop NT4 transport test, not Systemcore hardware qualification.
"""
import argparse
import base64
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
from importlib import metadata
import json
import os
from pathlib import Path
import queue
import socket
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parent
REPOSITORY = ROOT.parent.parent
sys.path.insert(0, str(REPOSITORY))
from custom_vision.publisher import Publisher, wire_packet  # noqa: E402


class Server:
    def __init__(self, runtime, port, topic):
        self.lines = queue.Queue()
        self.logs = []
        environment = dict(os.environ, DYLD_LIBRARY_PATH=runtime["native"])
        command = [runtime["java"], "-Xmx96m", "--enable-native-access=ALL-UNNAMED",
                   "-Djava.library.path=" + runtime["native"], "-cp", runtime["classpath"],
                   "Alpha7Server", str(port), topic, str(ROOT / ".cache" / "networktables.json")]
        self.process = subprocess.Popen(command, cwd=ROOT, env=environment, stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        text=True, encoding="utf-8", bufsize=1)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()
        try:
            self._response("READY", timeout=8)
        except Exception:
            self.close()
            raise

    def _read(self):
        for line in self.process.stdout:
            line = line.rstrip("\n")
            if line.startswith("CV_INTEROP\t"):
                self.lines.put(line.split("\t"))
            else:
                self.logs.append(line)

    def _response(self, expected, timeout=3):
        try:
            parts = self.lines.get(timeout=timeout)
        except queue.Empty:
            raise RuntimeError(f"No Java {expected} response; exit={self.process.poll()}; logs={self.logs[-10:]}")
        if parts[1] != expected:
            raise AssertionError(f"Expected {expected}, got {parts[1]}")
        return parts

    @staticmethod
    def decode(parts):
        return {"timestamp_ns": int(parts[2]), "server_time_ns": int(parts[3]),
                "now_ns": int(parts[4]), "value": base64.b64decode(parts[5]).decode("utf-8")}

    def request(self, command):
        self.process.stdin.write(command + "\n")
        self.process.stdin.flush()
        if command == "QUEUE":
            items = []
            while True:
                parts = self.lines.get(timeout=3)
                if parts[1] == "QUEUE_END":
                    return items
                if parts[1] != "ITEM":
                    raise AssertionError("Unexpected Java queue response")
                items.append(self.decode(parts))
        parts = self._response(command)
        if command in ("SNAPSHOT", "LATE"):
            return self.decode(parts)
        return {"connected": parts[2] == "true", "now_ns": int(parts[3])}

    def close(self):
        if self.process.poll() is None:
            try:
                self.request("CLOSE")
                self.process.wait(timeout=3)
            except Exception:
                self.process.terminate()
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2)


def eventually(test, description, timeout=8):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = test()
        if value:
            return value
        time.sleep(0.02)
    raise AssertionError(f"Timed out: {description}")


def free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def fresh(fixture, frame):
    payload = deepcopy(fixture)
    payload.update(frame_id=frame, connected=True, capture_monotonic_us=time.monotonic_ns() // 1000,
                   publish_unix_us=time.time_ns() // 1000)
    # Harness-only additive probes: no new permanent protocol fields.
    payload["interop_probe_integer"] = 9007199254740991
    payload["interop_probe_float"] = 1.123456789
    return payload


def encoded(payload):
    return json.dumps(wire_packet(payload), allow_nan=False, separators=(",", ":"))


def assert_units(snapshot, payload):
    value = json.loads(snapshot["value"])
    for field in ("capture_monotonic_us", "publish_unix_us", "capture_server_us"):
        assert type(value[field]) is int and value[field] == payload[field], field
    assert value["interop_probe_integer"] == 9007199254740991
    assert value["interop_probe_float"] == 1.123457
    assert snapshot["timestamp_ns"] > 0
    # NT4 wire timestamps have microsecond resolution, while alpha-7 API uses ns.
    assert snapshot["timestamp_ns"] % 1000 == 0
    assert abs(snapshot["now_ns"] - snapshot["timestamp_ns"]) < 2_000_000_000
    assert abs(snapshot["timestamp_ns"] - value["capture_server_us"] * 1000) < 2_000_000_000
    # serverTime may be 0/1 for a server-local view, as documented by alpha-7.
    assert snapshot["server_time_ns"] in (0, 1) or snapshot["server_time_ns"] % 1000 == 0


def run(fixture_path):
    import ntcore
    assert metadata.version("pyntcore") == "2024.3.2.1", "Use requirements.txt in an isolated environment"
    runtime = json.loads((ROOT / ".cache" / "runtime.json").read_text())
    assert runtime["wpilib_version"] == "2027.0.0-alpha-7"
    java_version = subprocess.run([runtime["java"], "-version"], text=True, capture_output=True,
                                  check=True, timeout=5).stderr.splitlines()
    fixture = json.loads(fixture_path.read_text())
    assert fixture.get("schema_version") == 2, "Expected deployed schema-2 producer fixture"
    pipeline = fixture["pipeline"]
    table = "/CustomVisionInterop"
    server = None
    publisher = None
    checks = []
    evidence = {}
    try:
        port = free_port()
        server = Server(runtime, port, f"{table}/{pipeline}/result")
        # Configure the native instance before connecting; no default controller
        # address, camera, or team discovery is contacted by this test.
        publisher = Publisher({"enabled": False, "table": table, "period_ms": 10})
        publisher.instance = ntcore.NetworkTableInstance.create()
        publisher.instance.setServer("127.0.0.1", port)
        publisher.instance.startClient4("CustomVisionInterop-pinned-2024")
        eventually(publisher.instance.isConnected, "Python producer connection")
        eventually(lambda: publisher.instance.getServerTimeOffset() is not None, "NT server clock offset")
        checks.append("pinned_python_connects_to_released_alpha7_java_server")

        offset = publisher.instance.getServerTimeOffset()
        python_now = ntcore._now()
        java_clock = server.request("STATUS")["now_ns"]
        assert type(offset) is int and type(python_now) is int
        assert abs((python_now + offset) * 1000 - java_clock) < 2_000_000_000
        evidence.update(python_offset_us=offset, python_nt_now_us=python_now, java_nt_now_ns=java_clock)
        checks.append("python_private_now_and_offset_are_microseconds_java_now_is_nanoseconds")

        packet = fresh(fixture, 123)
        publisher.publish(packet)
        expected = encoded(packet)
        snapshot = eventually(lambda: (item if (item := server.request("SNAPSHOT"))["value"] == expected else None),
                              "complete serialized Publisher string")
        assert packet["time_sync_valid"] is True
        assert_units(snapshot, packet)
        evidence.update(capture_server_us=packet["capture_server_us"], java_timestamp_ns=snapshot["timestamp_ns"],
                        java_server_time_ns=snapshot["server_time_ns"])
        checks.append("coherent_string_exact_integer_us_and_six_decimal_float_roundtrip")
        server.request("RETAIN")

        sent = {expected}
        for frame in range(124, 128):
            update = fresh(fixture, frame)
            publisher.publish(update)
            sent.add(encoded(update))
            time.sleep(0.02)
        latest = encoded(update)
        eventually(lambda: server.request("SNAPSHOT")["value"] == latest, "latest coherent publication")
        queue_values = server.request("QUEUE")
        assert queue_values, "Expected updates in Java subscriber queue"
        assert all(item["value"] in sent for item in queue_values), "Mixed or unknown packet content"
        assert all(type(json.loads(item["value"])["packet_seq"]) is int for item in queue_values)
        assert update["packet_seq"] > packet["packet_seq"]
        evidence["java_queue_count"] = len(queue_values)
        checks.append("queued_strings_are_whole_known_publications")

        server.request("STOP")
        eventually(lambda: not publisher.instance.isConnected(), "producer detects server disconnect")
        unsynchronized = fresh(fixture, 128)
        publisher.publish(unsynchronized)
        assert unsynchronized["capture_server_us"] is None and unsynchronized["time_sync_valid"] is False
        assert unsynchronized["packet_seq"] > update["packet_seq"]
        checks.append("disconnect_suppresses_capture_server_time_even_if_camera_connected")

        server.request("START")
        eventually(publisher.instance.isConnected, "producer reconnects")
        eventually(lambda: publisher.instance.getServerTimeOffset() is not None, "offset after reconnect")
        restored = fresh(fixture, 129)
        publisher.publish(restored)
        final = encoded(restored)
        eventually(lambda: server.request("SNAPSHOT")["value"] == final, "reconnected publication")
        assert restored["capture_server_us"] is not None and restored["time_sync_valid"] is True
        checks.append("reconnect_reacquires_clock_offset_and_publication")
        # Retention is deliberately configured by this harness, not Publisher.
        # A retained string carries its old sequence/time and is not liveness.
        server.request("RETAIN")
        publisher.close()
        eventually(lambda: not server.request("STATUS")["connected"], "Java detects source close")
        retained = server.request("LATE")
        assert retained["value"] == final
        assert json.loads(retained["value"])["packet_seq"] == restored["packet_seq"]
        checks.append("late_subscriber_receives_retained_old_packet_after_source_close")
        evidence["fixture_sha256"] = hashlib.sha256(fixture_path.read_bytes()).hexdigest()
        evidence["producer_files_sha256"] = {
            name: hashlib.sha256((REPOSITORY / name).read_bytes()).hexdigest()
            for name in ("custom_vision/publisher.py", "custom_vision/nt_clock.py")}
        producer_revision = subprocess.run(["git", "rev-parse", "HEAD"], cwd=REPOSITORY,
                                           text=True, capture_output=True, check=True, timeout=5).stdout.strip()
        revision_matches = all(
            hashlib.sha256(subprocess.run(["git", "show", f"{producer_revision}:{name}"],
                                         cwd=REPOSITORY, capture_output=True, check=True, timeout=5).stdout).hexdigest() == digest
            for name, digest in evidence["producer_files_sha256"].items())
        return {"status": "passed", "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
                "checks": checks, "evidence": evidence,
                "protocol_profile": restored["protocol_profile"],
                "producer_revision": producer_revision,
                "producer_revision_matches_recorded_source_hashes": revision_matches,
                "python_producer": "pyntcore==2024.3.2.1", "java_server": "org.wpilib.ntcore:2027.0.0-alpha-7",
                "java_runtime": java_version,
                "artifacts_lock_sha256": hashlib.sha256((ROOT / "artifacts.lock.json").read_bytes()).hexdigest(),
                "host": {"platform": sys.platform, "python": sys.version.split()[0]},
                "fixture": str(fixture_path.relative_to(REPOSITORY)) if fixture_path.is_relative_to(REPOSITORY) else fixture_path.name,
                "boundaries": ["desktop localhost only", "no camera/HAL/controller hardware",
                               "no exposure/capture-correction measurement", "no worst-case-load qualification"]}
    finally:
        if publisher is not None:
            publisher.close()
        if server is not None:
            server.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixture", type=Path, required=True, help="Canonical producer golden JSON fixture")
    parser.add_argument("--report", type=Path, default=ROOT / ".cache" / "result.json")
    args = parser.parse_args()
    try:
        report = run(args.fixture.resolve())
        exit_code = 0
    except (FileNotFoundError, ModuleNotFoundError) as error:
        report = {"status": "unrun", "reason": str(error)}
        exit_code = 2
    except Exception as error:
        report = {"status": "failed", "reason": f"{type(error).__name__}: {error}"}
        exit_code = 1
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
