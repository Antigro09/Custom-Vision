#!/usr/bin/env python3
"""Bound the camera-free native calibration integration check on macOS.

Each run owns a fresh process group and its observed descendant process groups,
including workers that start separate sessions. No unrelated group is signaled.
RSS is sampled, not a kernel-enforced memory limit or an exact peak. A new session
whose spawning ancestry disappears between snapshots can be missed, even if an
orphan remains alive; this is observed descendant cleanup, not kernel isolation.
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


def process_table():
    result = subprocess.run(['/bin/ps', '-axo', 'pid=,ppid=,pgid=,rss=,stat=,lstart=,comm='],
                            capture_output=True, text=True, check=True, timeout=5)
    members = []
    for line in result.stdout.splitlines():
        fields = line.split(None, 10)
        if len(fields) == 11:
            members.append(dict(pid=int(fields[0]), ppid=int(fields[1]),
                                pgid=int(fields[2]), started=' '.join(fields[5:10]),
                                rss_bytes=int(fields[3]) * 1024,
                                state=fields[4], command=fields[10]))
    if not members:
        raise subprocess.SubprocessError('Process table contained no parsable rows')
    return members


def process_group(pgid):
    return [member for member in process_table() if member['pgid'] == pgid]


class OwnedProcessGroups:
    """Discover groups by observed ancestry, then retain them across reparenting.

    Matching observed process start strings avoids using a reused PID as an
    ancestry anchor. An empty group is retired and cannot become owned again
    without new descendant ancestry. Existing members of an established group
    remain owned when its original leader exits; outsiders cannot join a process
    group in another session. No process-name or arbitrary parent-PID matching is
    used. Sampling cannot establish ownership of a new session whose spawning
    ancestry disappears before it is observed, even if that orphan persists.
    """
    def __init__(self, child_pid):
        self.root_pid = child_pid
        self.supervisor_pgid = os.getpgrp()
        if child_pid == self.supervisor_pgid:
            raise ValueError('The launched child must have its own process group')
        self.active_groups = {child_pid}
        self.seen_groups = {child_pid}
        self.identities = {}
        self.seen_pids = set()

    def sample(self, *, direct_child_reaped=False):
        table = process_table()
        rows = {row['pid']: row for row in table}
        owned = {pid for pid, started in self.identities.items()
                 if pid in rows and rows[pid]['started'] == started}
        # The unreaped direct PID cannot be reused; bind its first observed start.
        if not direct_child_reaped and self.root_pid in rows and self.root_pid not in self.identities:
            owned.add(self.root_pid)
        groups = set(self.active_groups)
        # Do not retain a group ID if its former leader PID has visibly been reused.
        for pgid in list(groups):
            if (pgid in rows and pgid in self.identities
                    and rows[pgid]['started'] != self.identities[pgid]):
                groups.remove(pgid)
        while True:
            before = (len(owned), len(groups))
            for row in table:
                if row['pgid'] == self.supervisor_pgid:
                    continue
                if row['pid'] in owned or row['ppid'] in owned or row['pgid'] in groups:
                    owned.add(row['pid'])
                    groups.add(row['pgid'])
            if before == (len(owned), len(groups)):
                break
        members = [row for row in table if row['pid'] in owned and row['pgid'] != self.supervisor_pgid]
        self.active_groups = {row['pgid'] for row in members}
        self.seen_groups.update(self.active_groups)
        for row in members:
            self.identities[row['pid']] = row['started']
            self.seen_pids.add(row['pid'])
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
    """Reap the direct child and check all observed, owned descendant groups."""
    started = time.monotonic()
    report = dict(command=command, wall_deadline_seconds=deadline,
                  term_grace_seconds=grace, sampled_peak_group_rss_bytes=0,
                  sampled_peak_artifact_bytes=0, seen_pids=[], signals=[],
                  rss_limit_bytes=2 * GIB, artifact_limit_bytes=ARTIFACT_LIMIT,
                  rss_measurement='sampled aggregate of launched process group and observed descendant groups; peak or unobserved ancestry may be missed',
                  sampling_interval_seconds=.1, signal_targets=[])
    child = None
    ownership = None
    usage = None
    reaped = False
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

    def send(signum, groups=None):
        # Groups are created by the launched child or established by observed
        # descendant ancestry. Always exclude the supervisor's own group.
        for pgid in sorted(ownership.active_groups if groups is None else groups, reverse=True):
            if pgid == ownership.supervisor_pgid or pgid not in ownership.seen_groups:
                raise ValueError('Refusing to signal a group without owned descendant provenance')
            try:
                os.killpg(pgid, signum)
                name = signal.Signals(signum).name
                report['signals'].append(name)
                report['signal_targets'].append(dict(pgid=pgid, signal=name))
            except ProcessLookupError:
                pass
            except OSError as exc:
                report.setdefault('cleanup_signal_errors', []).append(f'{type(exc).__name__}: {exc}')

    def record_sample(members):
        rss = sum(m['rss_bytes'] for m in members)
        size = artifact_bytes(directory)
        report['sampled_peak_group_rss_bytes'] = max(report['sampled_peak_group_rss_bytes'], rss)
        report['sampled_peak_artifact_bytes'] = max(report['sampled_peak_artifact_bytes'], size)
        return rss, size

    previous = {s: signal.signal(s, cancel) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        with (directory / 'pytest.log').open('w') as output:
            child = subprocess.Popen(command, cwd=ROOT, env=environment,
                                     stdout=output, stderr=subprocess.STDOUT,
                                     start_new_session=True)
            ownership = OwnedProcessGroups(child.pid)
            report['pid'] = report['pgid'] = child.pid
            save(directory / 'run-start.json', report)
            print(f'Owned native process group {child.pid}; artifacts {directory}', flush=True)
            while True:
                members = ownership.sample(direct_child_reaped=reaped)
                rss, size = record_sample(members)
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
                time.sleep(.1)
    except BaseException as exc:
        report['stop_reason'] = 'supervisor_error'
        report['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        if child is not None and ownership is not None:
            # Also clean up descendants after an early pytest/per-fit failure.
            def monitor_cleanup():
                try:
                    observed = ownership.sample(direct_child_reaped=reaped)
                    record_sample(observed)
                    return observed
                except (subprocess.SubprocessError, OSError) as exc:
                    report.setdefault('cleanup_monitor_errors', []).append(f'{type(exc).__name__}: {exc}')
                    return None

            members = monitor_cleanup()
            # A failed monitor cannot prevent signaling our known, owned group.
            if members or members is None or not reaped:
                send(signal.SIGTERM)
                term_groups = set(ownership.active_groups)
                end = time.monotonic() + grace
                while time.monotonic() < end:
                    reap()
                    members = monitor_cleanup()
                    newly_owned = ownership.active_groups - term_groups
                    if newly_owned:
                        send(signal.SIGTERM, newly_owned)
                        term_groups.update(newly_owned)
                    if members == []:
                        break
                    time.sleep(.1)
                members = monitor_cleanup()
                if members or members is None:
                    send(signal.SIGKILL)
            # Allow the OS to reap orphaned descendants, while retaining evidence.
            for _ in range(50):
                reap()
                leftovers = monitor_cleanup()
                if reaped and (leftovers == [] or leftovers is None):
                    break
                time.sleep(.1)
            report.update(exit_code=child.returncode, direct_child_reaped=reaped,
                          remaining_group_members=leftovers)
        for signum, handler in previous.items():
            signal.signal(signum, handler)
        report['wall_seconds'] = time.monotonic() - started
        report['seen_pids'] = sorted(ownership.seen_pids) if ownership else []
        report['seen_pgids'] = sorted(ownership.seen_groups) if ownership else []
        report['remaining_owned_pgids'] = sorted(ownership.active_groups) if ownership else []
        report['sampled_peak_owned_rss_bytes'] = report['sampled_peak_group_rss_bytes']
        report['descendant_scope'] = 'launched pytest plus ancestry-observed descendant process groups retained across reparenting; sessions whose spawning ancestry disappears between snapshots can be missed even if an orphan persists'
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
    parser.add_argument('--guided', action='store_true', help='exercise the explicit guided capture worker and image-group validation')
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
        # Match the real job adapter: a worker starts a new session. Make its
        # grandchild start a third session too, then retain both after reparenting.
        grandchild_code = 'import os,time; print("nested grandchild",os.getpid(),os.getpgrp(),flush=True); time.sleep(60)'
        worker_code = ('import os,subprocess,sys,time; '
                       'print("nested worker",os.getpid(),os.getpgrp(),flush=True); '
                       'subprocess.Popen([sys.executable,"-c",' + repr(grandchild_code) + '],start_new_session=True); '
                       'time.sleep(60)')
        for name, lifetime, deadline in (('nested_deadline', 60, .8), ('nested_early_exit', .6, 10)):
            case = directory / name
            case.mkdir()
            nested_code = ('import subprocess,sys,time; '
                           'subprocess.Popen([sys.executable,"-c",' + repr(worker_code) + '],start_new_session=True); '
                           f'time.sleep({lifetime})')
            report = supervise([str(args.python), '-c', nested_code], environment, case, deadline, grace=.5)
            assert report['direct_child_reaped'] and report['remaining_group_members'] == [], report
            assert len(report['seen_pgids']) >= 3 and len(report['seen_pids']) >= 3, report
            assert report['remaining_owned_pgids'] == [] and report['sampled_peak_owned_rss_bytes'] > 0, report
            assert report['stop_reason'] == ('wall_deadline' if lifetime == 60 else 'child_completed'), report
            assert os.getpgrp() not in report['seen_pgids'], report
            assert all(target['pgid'] in report['seen_pgids'] and target['pgid'] != os.getpgrp()
                       for target in report['signal_targets']), report
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
        original_monitor = globals()['process_table']
        calls = 0
        def fail_first_monitor():
            nonlocal calls
            calls += 1
            if calls == 1:
                raise subprocess.TimeoutExpired('injected ps failure', 5)
            return original_monitor()
        globals()['process_table'] = fail_first_monitor
        try:
            report = supervise([str(args.python), '-c', code + 'time.sleep(60)'], environment, case, 10, grace=.5)
        finally:
            globals()['process_table'] = original_monitor
        assert report['stop_reason'] == 'supervisor_error' and report['direct_child_reaped']
        assert report['remaining_group_members'] == [] and report['signals']
        print(f'PASS: deadline, early-exit, nested-session deadline/early-exit, cancellation and monitor-error cleanup; {directory}', flush=True)
        return 0
    if args.guided:
        environment['CUSTOM_VISION_GUIDED_NATIVE'] = '1'
    selected_test = 'tests/test_calibration_guided_native.py::test_guided_native_worker_retains_exact_diagnostics_and_provenance' if args.guided else TEST
    code = ("import cv2,mrcal,scipy.optimize,pytest,json; from threadpoolctl import threadpool_info; "
            "cv2.setNumThreads(0); cv2.ocl.setUseOpenCL(False); assert cv2.getNumThreads()==1; "
            "print(json.dumps({'opencv_threads':cv2.getNumThreads(),'opencl':cv2.ocl.useOpenCL(),"
            "'pools':threadpool_info()}),flush=True); raise SystemExit(pytest.main(" +
            repr(['-q', '-rs', '-s', '--basetemp=' + str(directory / 'pytest-tmp'),
                  '-o', 'cache_dir=' + str(directory / 'pytest-cache'),
                  '--junitxml=' + str(directory / 'junit.xml'), selected_test]) + '))')
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
                 and not report.get('cleanup_signal_errors')
                 and report.get('native_test_passed')) else 1


if __name__ == '__main__':
    raise SystemExit(main())
