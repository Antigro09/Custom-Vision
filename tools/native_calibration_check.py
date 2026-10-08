#!/usr/bin/env python3
"""Bound the camera-free native calibration integration check on macOS.

Each run owns a fresh process group and artifact directory. No external process
is signaled. RSS is sampled, not a kernel-enforced memory limit or an exact peak.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
TEST = 'tests/test_guided_calibration.py::test_real_mrcal_full_solve_validation_uncertainty_and_runtime_export'
GIB = 1024 ** 3
ARTIFACT_LIMIT = 256 * 1024 ** 2


def process_group(pgid):
    result = subprocess.run(['/bin/ps', '-axo', 'pid=,ppid=,pgid=,rss=,stat=,comm='],
                            capture_output=True, text=True, check=True, timeout=5)
    members = []
    for line in result.stdout.splitlines():
        fields = line.split(None, 5)
        if len(fields) == 6 and int(fields[2]) == pgid:
            members.append(dict(pid=int(fields[0]), ppid=int(fields[1]),
                                rss_bytes=int(fields[3]) * 1024,
                                state=fields[4], command=fields[5]))
    return members


def available_memory():
    output = subprocess.run(['/usr/bin/vm_stat'], capture_output=True, text=True,
                            check=True, timeout=5).stdout
    page_size = int(re.search(r'page size of (\d+) bytes', output).group(1))
    free_pages = int(re.search(r'Pages free:\s+(\d+)', output).group(1))
    return page_size * free_pages


def artifact_bytes(directory):
    return sum(p.stat().st_size for p in directory.rglob('*')
               if not p.is_symlink() and p.is_file())


def save(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n')


def supervise(command, environment, directory, deadline, grace=5):
    """Return a report after reaping the direct child and checking its group."""
    started = time.monotonic()
    report = dict(command=command, wall_deadline_seconds=deadline,
                  term_grace_seconds=grace, sampled_peak_group_rss_bytes=0,
                  sampled_peak_artifact_bytes=0, seen_pids=[], signals=[],
                  rss_limit_bytes=2 * GIB, artifact_limit_bytes=ARTIFACT_LIMIT,
                  rss_measurement='sampled aggregate of owned process group; peak may be missed')
    child = None
    usage = None
    reaped = False
    seen = set()
    cancelled = False

    def cancel(signum, _frame):
        nonlocal cancelled
        cancelled = True
        report['cancel_signal'] = signum

    def reap():
        nonlocal reaped, usage
        if child is not None and not reaped:
            pid, status, observed_usage = os.wait4(child.pid, os.WNOHANG)
            if pid:
                reaped = True
                usage = observed_usage
                child.returncode = os.waitstatus_to_exitcode(status)

    def send(signum):
        # start_new_session creates this group; we never signal other groups.
        try:
            os.killpg(child.pid, signum)
            report['signals'].append(signal.Signals(signum).name)
        except ProcessLookupError:
            pass

    previous = {s: signal.signal(s, cancel) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        with (directory / 'pytest.log').open('w') as output:
            child = subprocess.Popen(command, cwd=ROOT, env=environment,
                                     stdout=output, stderr=subprocess.STDOUT,
                                     start_new_session=True)
            report['pid'] = report['pgid'] = child.pid
            save(directory / 'run-start.json', report)
            print(f'Owned native process group {child.pid}; artifacts {directory}', flush=True)
            while True:
                members = process_group(child.pid)
                seen.update(m['pid'] for m in members)
                rss = sum(m['rss_bytes'] for m in members)
                size = artifact_bytes(directory)
                report['sampled_peak_group_rss_bytes'] = max(report['sampled_peak_group_rss_bytes'], rss)
                report['sampled_peak_artifact_bytes'] = max(report['sampled_peak_artifact_bytes'], size)
                reap()
                if reaped:
                    report['stop_reason'] = 'child_completed'
                    break
                reason = ('cancelled' if cancelled else
                          'wall_deadline' if time.monotonic() - started >= deadline else
                          'unexpected_rss_over_estimate' if rss > 2 * GIB else
                          'unexpected_artifacts_over_estimate' if size > ARTIFACT_LIMIT else None)
                if reason:
                    report['stop_reason'] = reason
                    break
                time.sleep(.5)
    except BaseException as exc:
        report['stop_reason'] = 'supervisor_error'
        report['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        if child is not None:
            # Also clean up descendants after an early pytest/per-fit failure.
            def monitor_cleanup():
                try:
                    return process_group(child.pid)
                except (subprocess.SubprocessError, OSError) as exc:
                    report.setdefault('cleanup_monitor_errors', []).append(f'{type(exc).__name__}: {exc}')
                    return None

            members = monitor_cleanup()
            # A failed monitor cannot prevent signaling our known, owned group.
            if members or members is None or not reaped:
                send(signal.SIGTERM)
                end = time.monotonic() + grace
                while time.monotonic() < end:
                    reap()
                    members = monitor_cleanup()
                    if members == []:
                        break
                    time.sleep(.1)
                members = monitor_cleanup()
                if members or members is None:
                    send(signal.SIGKILL)
            if not reaped:
                _, status, usage = os.wait4(child.pid, 0)
                reaped = True
                child.returncode = os.waitstatus_to_exitcode(status)
            # Allow the OS to reap orphaned descendants, while retaining evidence.
            for _ in range(50):
                leftovers = monitor_cleanup()
                if leftovers == [] or leftovers is None:
                    break
                time.sleep(.1)
            report.update(exit_code=child.returncode, direct_child_reaped=reaped,
                          remaining_group_members=leftovers)
        for signum, handler in previous.items():
            signal.signal(signum, handler)
        report['wall_seconds'] = time.monotonic() - started
        report['seen_pids'] = sorted(seen)
        if usage is not None:
            report.update(user_cpu_seconds=usage.ru_utime, system_cpu_seconds=usage.ru_stime,
                          wait4_maxrss_bytes=usage.ru_maxrss,
                          wait4_scope='direct child and usage of descendants it waited for; macOS maxrss is bytes')
        report['final_artifact_bytes_before_receipt'] = artifact_bytes(directory)
        save(directory / 'resources.json', report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--python', type=Path, default=ROOT / 'data/native-mrcal-mac-env/bin/python')
    parser.add_argument('--deadline', type=float, default=1800)
    parser.add_argument('--self-test', action='store_true', help='only short sleeping-process cleanup checks')
    args = parser.parse_args()
    if sys.platform != 'darwin':
        parser.error('resource units and memory preflight require macOS')
    if not 0 < args.deadline <= 1800:
        parser.error('deadline must be positive and at most 1800 seconds')
    directory = ROOT / 'data/native-calibration-runs' / str(uuid.uuid4())
    directory.mkdir(parents=True)
    memory = available_memory()
    disk = shutil.disk_usage(directory).free
    process_group(os.getpgrp())  # Fail before launching if monitoring is denied.
    save(directory / 'preflight.json', dict(free_memory_bytes=memory, free_disk_bytes=disk,
                                           required_free_memory_bytes=2 * GIB,
                                           required_free_disk_bytes=ARTIFACT_LIMIT,
                                           synthetic_only=True, requested_compute_threads=1))
    if memory < 2 * GIB or disk < ARTIFACT_LIMIT:
        raise SystemExit('Insufficient available resources; no child started')
    environment = os.environ.copy()
    # Keep the venv bin path: resolving its Python symlink can hide its CLI.
    environment.update(PATH=str(args.python.absolute().parent) + os.pathsep + environment.get('PATH', ''),
                       PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', VISION_TEST_GUI='0', PYTHONDONTWRITEBYTECODE='1',
                       OPENCV_FOR_THREADS_NUM='1', OPENCV_OPENCL_RUNTIME='disabled',
                       OMP_NUM_THREADS='1', OMP_THREAD_LIMIT='1', OMP_DYNAMIC='FALSE',
                       OPENBLAS_NUM_THREADS='1', VECLIB_MAXIMUM_THREADS='1', MKL_NUM_THREADS='1',
                       BLIS_NUM_THREADS='1', NUMEXPR_NUM_THREADS='1')
    if args.self_test:
        # Test deadline and early-direct-child exit with a live grandchild.
        for name, suffix, deadline in (('deadline', 'time.sleep(60)', .8), ('early_exit', '', 10)):
            case = directory / name
            case.mkdir()
            code = ('import subprocess,sys,time; '
                    'subprocess.Popen([sys.executable,"-c","import time; time.sleep(60)"]); ' + suffix)
            report = supervise([str(args.python), '-c', code], environment, case, deadline, grace=.5)
            assert report['direct_child_reaped'] and not report['remaining_group_members'], report
            assert report['signals'] and report['stop_reason'] == ('wall_deadline' if suffix else 'child_completed')
        import threading
        case = directory / 'cancellation'
        case.mkdir()
        timer = threading.Timer(.8, lambda: os.kill(os.getpid(), signal.SIGTERM))
        timer.start()
        try:
            report = supervise([str(args.python), '-c', code + 'time.sleep(60)'], environment, case, 10, grace=.5)
        finally:
            timer.cancel()
        assert report['stop_reason'] == 'cancelled' and report['direct_child_reaped']
        assert report['remaining_group_members'] == []
        case = directory / 'monitor_failure'
        case.mkdir()
        original_monitor = globals()['process_group']
        calls = 0
        def fail_first_monitor(pgid):
            nonlocal calls
            calls += 1
            if calls == 1:
                raise subprocess.TimeoutExpired('injected ps failure', 5)
            return original_monitor(pgid)
        globals()['process_group'] = fail_first_monitor
        try:
            report = supervise([str(args.python), '-c', code + 'time.sleep(60)'], environment, case, 10, grace=.5)
        finally:
            globals()['process_group'] = original_monitor
        assert report['stop_reason'] == 'supervisor_error' and report['direct_child_reaped']
        assert report['remaining_group_members'] == [] and report['signals']
        print(f'PASS: deadline, early-exit, cancellation and monitor-error cleanup; {directory}', flush=True)
        return 0
    code = ("import cv2,mrcal,scipy.optimize,pytest,json; from threadpoolctl import threadpool_info; "
            "cv2.setNumThreads(0); cv2.ocl.setUseOpenCL(False); assert cv2.getNumThreads()==1; "
            "print(json.dumps({'opencv_threads':cv2.getNumThreads(),'opencl':cv2.ocl.useOpenCL(),"
            "'pools':threadpool_info()}),flush=True); raise SystemExit(pytest.main(" +
            repr(['-q', '-rs', '-s', '--basetemp=' + str(directory / 'pytest-tmp'),
                  '-o', 'cache_dir=' + str(directory / 'pytest-cache'),
                  '--junitxml=' + str(directory / 'junit.xml'), TEST]) + '))')
    report = supervise([str(args.python), '-c', code], environment, directory, args.deadline)
    try:
        cases = ET.parse(directory / 'junit.xml').getroot().findall('.//testcase')
        passed = len(cases) == 1 and not any(c.find(t) is not None for c in cases for t in ('skipped', 'failure', 'error'))
        calibration_reports = list((directory / 'pytest-tmp').rglob('report.json'))
        if len(calibration_reports) != 1:
            raise ValueError('Expected exactly one native calibration report')
        calibration = json.loads(calibration_reports[0].read_text())
        solver_logs = list(calibration_reports[0].parent.glob('*/*/solver.log'))
        passed &= (calibration['status'] != 'failed' and set(calibration['models']) == {'opencv8', 'spline'}
                   and len(solver_logs) == 4 and all(p.stat().st_size for p in solver_logs))
        report.update(native_test_passed=bool(passed), junit_test_count=len(cases),
                      calibration_report=str(calibration_reports[0]),
                      solver_logs=[str(p) for p in solver_logs])
    except (ET.ParseError, OSError, ValueError, KeyError) as exc:
        report.update(native_test_passed=False, evidence_error=f'{type(exc).__name__}: {exc}')
    save(directory / 'resources.json', report)
    print(json.dumps(report, indent=2), flush=True)
    return 0 if (report.get('exit_code') == 0 and report.get('stop_reason') == 'child_completed'
                 and report.get('remaining_group_members') == []
                 and not report.get('cleanup_monitor_errors')
                 and report.get('native_test_passed')) else 1


if __name__ == '__main__':
    raise SystemExit(main())
