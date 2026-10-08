"""Opt-in actual mrcal worker integration using explicitly synthetic evidence.

Enable only in the isolated native environment with CUSTOM_VISION_GUIDED_NATIVE=1.
The normal suite does not render this fixture or launch native fits. This checks
the desktop job/diagnostics/export pipeline, not physical camera accuracy,
exposure timing, calibration trust or a robot mount.
"""
import copy
from dataclasses import asdict
import json
import os
import shutil
import sys
import time

import numpy as np
import pytest

from custom_vision.calibration_capture import CaptureSource
from custom_vision.calibration_guided import GuidedCalibrationJobs
from custom_vision.calibration_review import require_candidate_review
from custom_vision.calibration_session import grid_coverage, write_json
from test_guided_calibration import rendered_boards, make_solver_session


pytestmark = pytest.mark.skipif(
    os.environ.get("CUSTOM_VISION_GUIDED_NATIVE") != "1",
    reason="Guided actual-native test is opt-in; enable CUSTOM_VISION_GUIDED_NATIVE=1 in the isolated native environment",
)


@pytest.fixture(scope="module", autouse=True)
def guided_native_ready():
    # Autouse readiness runs before the shared module-scoped raster fixture.
    pytest.importorskip("mrcal", reason="Actual native mrcal is required")
    if shutil.which("mrcal-calibrate-cameras") is None:
        pytest.skip("Actual mrcal-calibrate-cameras command is unavailable")


def _wait_terminal(manager, identifier):
    deadline = time.monotonic() + 125.
    while time.monotonic() < deadline:
        job = manager.get_job(identifier)
        if job["status"] in ("completed", "failed", "canceled", "unavailable"):
            return job
        time.sleep(.1)
    manager.cancel_job(identifier)
    pytest.fail("Guided native job did not reach a terminal state inside its bounded 120-second budget")


def test_guided_native_worker_retains_exact_diagnostics_and_provenance(rendered_boards, tmp_path):
    board, expected_matrix, _, _ = rendered_boards
    fixture_directory = make_solver_session(tmp_path / "seed-fixture", rendered_boards)
    fixture = json.loads((fixture_directory / "session.json").read_text())
    mode = {"width": 640, "height": 480, "crop": None, "binning": None,
            "focus": {"kind": "unknown", "value": None, "locked": False}}
    source = CaptureSource(
        "native_synthetic_fixture", "Synthetic native integration fixture (no camera)", "synthetic",
        lambda _: pytest.fail("This native solve test must never open a capture source"),
        camera={"physical_id": None}, mode=mode,
    )
    manager = GuidedCalibrationJobs(tmp_path / "guided-store", sources=[source],
                                    python=sys.executable, solver_python=sys.executable)
    try:
        session = manager.create_session({
            "board": asdict(board),
            "camera": {"physical_id": None, "user_label": "Synthetic native fixture",
                       "identity_source": "offline_metadata", "capture_history": "Generated synthetic chessboard rasters; no sensor"},
            "mode": copy.deepcopy(mode), "input": {"kind": "capture", "source_id": source.source_id},
        })
        identifier = session["session_id"]
        directory = manager._directory("sessions", identifier) / "capture"
        shutil.copytree(fixture_directory, directory)
        record = manager._session_record(identifier)
        coverage = np.zeros((6, 8), bool)
        for view in fixture["views"]:
            coverage |= grid_coverage(view["corners"], 640, 480)
        record.update(board=asdict(board), width=640, height=480, views=copy.deepcopy(fixture["views"]),
                      status="selected", capture_revision=60, source_kind="synthetic", synthetic=True,
                      mode_dimensions_source="decoded", selected_directory=str(directory.relative_to(manager.root)))
        record["summary"].update(
            accepted_views=60, processed_frames=60, rejected_views=0,
            coverage_grid=coverage.astype(int).tolist(), coverage_fraction=float(coverage.mean()),
            timestamp_source="synthetic_fixture_sequence_not_capture_time", preview_artifact=None,
        )
        write_json(directory / "session.json", manager._saved_data(record, complete=True))
        write_json(manager._directory("sessions", identifier) / "record.json", record)

        assert manager.capture_status()["state"] == "idle"
        job = manager.create_job({
            "session_id": identifier, "operation": "solve", "solver": "mrcal",
            "options": {"min_views": 40, "fov_deg": 60., "max_validation_rms": .6,
                        "timeout_s": 120., "corner_detector": "opencv-sb", "no_spline": False},
        })
        terminal = _wait_terminal(manager, job["job_id"])
        assert terminal["status"] == "completed", terminal
        request = json.loads((manager._directory("jobs", job["job_id"]) / "request.json").read_text())
        assert request["validation_basis"] == "image_groups"
        candidate_id = terminal["result"]["candidate_id"]
        candidate = manager.get_candidate(candidate_id)
        report, calibration = candidate["report"], candidate["calibration"]
        assert candidate["quality_status"] in ("needs_review", "heuristics_passed_not_hardware_validated")
        assert report["status"] == candidate["quality_status"] == calibration["quality_status"]
        assert report["validation_basis"] == report["validation"]["basis"] == calibration["validation_basis"] == "image_groups"
        assert "capture timing unknown" in report["validation"]["method"]
        assert set(report["models"]) == {"opencv8", "spline"}
        for model in report["models"].values():
            assert model["holdout"]["rms_px"] < .6
            assert model["uncertainty"] is not None, model.get("uncertainty_error")
            assert np.isfinite(model["uncertainty"]["p95_px"])
        assert "model_difference" in report, report.get("model_difference_error")
        assert len(calibration["dist_coeffs"]) == 8
        assert calibration["lensmodel"] == "LENSMODEL_OPENCV8"
        matrix = np.asarray(calibration["camera_matrix"])
        assert matrix[0, 0] == pytest.approx(expected_matrix[0, 0], rel=.035)
        assert matrix[1, 1] == pytest.approx(expected_matrix[1, 1], rel=.035)
        assert calibration["robot_mount_calibrated"] is False

        pose_artifact = manager.get_artifact(candidate_id, "board-poses.json")
        poses = json.loads(pose_artifact["body"])
        assert len(poses["poses"]) == 60 and poses["robot_to_camera"] is None
        assert all(p["observed_corner_source"] == "solver_final_detection" for p in poses["poses"])
        assert all(np.asarray(p["observed_corners_px"]).shape == (35, 2) for p in poses["poses"])
        diagnostics = manager.diagnostics(candidate_id)
        assert diagnostics["status"] == "available" and diagnostics["frame"] == "camera_optical"
        assert diagnostics["units"] == "m" and len(diagnostics["frames"]) == 60
        assert diagnostics["board"] == asdict(board)
        assert {frame["image"] for frame in diagnostics["frames"]} == {view["image"] for view in fixture["views"]}
        assert all(np.isfinite(frame["reprojection_rms_px"]) and frame["reprojection_rms_px"] < 10.
                   for frame in diagnostics["frames"])
        assert all(np.all(np.asarray(frame["corners_camera_m"])[:, 2] > 0) for frame in diagnostics["frames"])
        assert not any("lack retained final detector" in warning for warning in diagnostics["warnings"])
        json.dumps(diagnostics, allow_nan=False)

        downloaded = json.loads(manager.get_artifact(candidate_id, report["intrinsics_file"])["body"])
        downloaded_report = json.loads(manager.get_artifact(candidate_id, "report.json")["body"])
        assert downloaded == calibration
        assert downloaded["synthetic"] is True and downloaded["source_kind"] == "synthetic"
        assert downloaded["calibration_provenance"]["capture_revision"] == 60
        assert downloaded["calibration_provenance"]["source_kind"] == "synthetic"
        assert downloaded_report["synthetic"] is True and downloaded_report["source_kind"] == "synthetic"
        for model in report["models"].values():
            assert manager.get_artifact(candidate_id, model["model"])["body"]
        assert candidate["synthetic"] is True and candidate["source_kind"] == "synthetic"
        assert candidate["capture_revision"] == 60 and candidate["stale"] is False
        assert candidate["review_status"]["activation_provenance_available"] is False
        assert candidate["review_status"]["physical_verification"] is False
        assert candidate["reviewed"] is False
        if candidate["quality_status"] == "needs_review":
            assert candidate["review_status"]["review_allowed"] is False
            with pytest.raises(ValueError, match="quality|investigation|issues"):
                manager.review(candidate_id, {"confirmed": True})
        else:
            reviewed = manager.review(candidate_id, {"confirmed": True})
            assert reviewed["reviewed"] is True
            assert reviewed["review_status"]["activation_provenance_available"] is False
        with pytest.raises(ValueError, match="Synthetic|quality|investigation|issues"):
            require_candidate_review(manager.candidate_data(candidate_id), manager.candidate_metadata(candidate_id))
        assert manager.capture_status()["state"] == "idle"
    finally:
        manager.close()
