"""Guided solver contract checks with stub fits; no native solver execution."""
import json
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from custom_vision import calibration_solver as solver


def test_time_block_default_is_unchanged():
    views = [{"source_time_s": i * .7} for i in range(90)]
    default = solver.split_views(views)
    assert default == solver.split_views(views, validation_basis="time_blocks")
    train, held = default
    assert set(train).isdisjoint(held)
    assert sorted(train + held) == list(range(90))
    assert {int(views[i]["source_time_s"] // 3) for i in train}.isdisjoint(
        int(views[i]["source_time_s"] // 3) for i in held)


@pytest.mark.parametrize("count", [25, 26, 40, 90, 180])
def test_image_groups_are_deterministic_whole_sequential_groups_without_timestamps(count):
    # Deliberately omit time and filename: group membership uses selected order,
    # without manufacturing temporal provenance from lexical image names.
    views = [{} for _ in range(count)]
    train, held = solver.split_views(views, validation_basis="image_groups")
    assert (train, held) == solver.split_views(views, validation_basis="image_groups")
    assert set(train).isdisjoint(held)
    assert sorted(train + held) == list(range(count))
    assert train == sorted(train) and held == sorted(held)
    assert len(train) >= 10 and len(held) >= max(6, int(np.ceil(count * .2)))
    assert {i // 5 for i in train}.isdisjoint(i // 5 for i in held)


def test_image_group_membership_does_not_use_untrusted_or_fabricated_times():
    absent = [{} for _ in range(40)]
    invalid = [{"source_time_s": float("nan")} for _ in range(40)]
    assert solver.split_views(absent, validation_basis="image_groups") == solver.split_views(
        invalid, validation_basis="image_groups")
    with pytest.raises(ValueError, match="timestamp"):
        solver.split_views(absent)
    with pytest.raises(ValueError, match="timestamp"):
        solver.split_views(invalid)


@pytest.mark.parametrize("count", [0, 5, 20])
def test_image_groups_require_five_groups(count):
    with pytest.raises(ValueError, match="five sequential image groups"):
        solver.split_views([{}] * count, validation_basis="image_groups")


def test_unknown_validation_basis_is_not_silently_replaced():
    with pytest.raises(ValueError, match="validation_basis"):
        solver.split_views([{}] * 40, validation_basis="exposure_time")


def _pose_model(count):
    frames = np.zeros((count, 6))
    frames[:, 5] = 1.
    mapping = np.array([[index, 0, -1] for index in range(count)], dtype=int)
    return SimpleNamespace(optimization_inputs=lambda: {
        "rt_ref_frame": frames, "indices_frame_camintrinsics_camextrinsics": mapping})


def test_board_pose_retains_actual_selected_solve_corner_order():
    native_corners = [[float(i + 1), float(i + 2)] for i in range(9)]
    unused = {"image": "frames/unused.png", "corners": [[900., 900.]]}
    selected = {"image": "frames/selected.png", "corners": native_corners}
    result = solver.board_poses(_pose_model(1), [1], [unused, selected])
    pose = result["poses"][0]
    assert pose["image"] == selected["image"]
    assert pose["observed_corners_px"] == native_corners
    assert pose["observed_corner_source"] == "solver_final_detection"
    assert pose["camera_cv_T_board"][2][3] == 1.
    assert result["robot_to_camera"] is None
    json.dumps(result, allow_nan=False)
    native_corners[0][0] = 200.
    assert pose["observed_corners_px"][0][0] == 1.  # Independent retained artifact.


def test_legacy_board_pose_fixture_without_corners_remains_supported():
    result = solver.board_poses(_pose_model(1), [0], [{"image": "frames/legacy.png"}])
    assert "observed_corners_px" not in result["poses"][0]
    assert "observed_corner_source" not in result["poses"][0]


@pytest.mark.parametrize("corners", [None, [], 1., np.array(1.), [1., 2.], [[1., 2., 3.]],
                                     [[1., float("nan")]], [[1., float("inf")]], [[False, 1.]],
                                     [["1", 2.]], [[1., 2.]] * 1601])
def test_present_invalid_final_corner_observations_fail_without_legacy_fallback(corners):
    with pytest.raises(ValueError, match="corner observations"):
        solver.board_poses(_pose_model(1), [0], [{"image": "frames/bad.png", "corners": corners}])


@pytest.fixture
def stubbed_calibration(tmp_path, monkeypatch):
    board = {"cols": 3, "rows": 3, "square_size_m": .03}
    corners = [[float(x + 2), float(y + 2)] for y in range(3) for x in range(3)]
    views = []
    for index in range(30):
        image = f"frames/frame{index:06d}.png"
        path = tmp_path / image
        path.parent.mkdir(exist_ok=True)
        path.write_bytes(b"stub image hash evidence, never decoded")
        views.append({"image": image, "source_frame_id": index,
                      "source_time_s": index * .7, "corners": corners, "weights": [1.] * 9})
    data = {"board": board, "width": 32, "height": 24, "views": views}
    monkeypatch.setitem(sys.modules, "mrcal", SimpleNamespace(__version__="stub-unit-test"))
    monkeypatch.setattr(solver, "load_session", lambda _: (tmp_path, data))
    monkeypatch.setattr(solver, "final_observations", lambda *_: (views, {"detector": "stub-test"}))
    monkeypatch.setattr(solver, "grid_coverage", lambda *_: np.ones((6, 8), bool))
    calls = []

    def stub_solve(_mrcal, _session, _data, _views, indices, _lensmodel, directory, _focal, _timeout):
        calls.append(list(indices))
        model = _pose_model(len(indices))
        inputs = model.optimization_inputs()
        inputs["observations_board"] = np.ones((len(indices), 3, 3, 3))
        model.optimization_inputs = lambda: inputs
        return model, directory / "stub.cameramodel"

    monkeypatch.setattr(solver, "solve_model", stub_solve)
    monkeypatch.setattr(solver, "validate_holdout", lambda *_: {"rms_px": .1, "views": []})
    monkeypatch.setattr(solver, "uncertainty_report", lambda *_: {"p95_px": .1})
    monkeypatch.setattr(solver, "export_opencv", lambda *_: {
        "width": 32, "height": 24, "board": board,
        "camera_matrix": [[20., 0., 16.], [0., 20., 12.], [0., 0., 1.]],
        "dist_coeffs": [0.] * 8, "lensmodel": "LENSMODEL_OPENCV8"})
    args = SimpleNamespace(session=tmp_path, corner_detector="opencv-sb", timeout=1., min_views=20,
                           fov_deg=60., no_spline=True, max_validation_rms=.6)
    return tmp_path, args, data, calls


@pytest.mark.parametrize("basis", [None, "time_blocks", "image_groups"])
def test_calibration_report_and_runtime_export_identify_validation_basis(stubbed_calibration, basis):
    session, args, data, calls = stubbed_calibration
    if basis is not None:
        args.validation_basis = basis
    if basis == "image_groups":
        for view in data["views"]:
            del view["source_time_s"]
    assert solver.calibrate(args) == 0
    latest = json.loads((session / "latest-solve.json").read_text())
    output = session / latest["directory"]
    report = json.loads((output / "report.json").read_text())
    expected = basis or "time_blocks"
    assert report["validation_basis"] == expected
    assert report["validation"]["basis"] == expected
    assert report["status"] == "heuristics_passed_not_hardware_validated"
    assert len(calls) == 2  # Stub training/full calls only; no native computation.
    exported = json.loads((output / report["intrinsics_file"]).read_text())
    assert exported["validation_basis"] == expected
    assert exported["quality_status"] == report["status"]
    if expected == "image_groups":
        assert report["validation"]["group_size_images"] == 5
        assert "capture timing unknown" in report["validation"]["method"]
        assert any("do not establish temporal independence" in w for w in report["warnings"])
        assert not any("3-second" in str(value) for value in report["validation"].values())
    else:
        assert report["validation"]["method"] == "deterministic 3-second-block holdout; not independent recapture"
        assert "group_size_images" not in report["validation"]
    poses = json.loads((output / "board-poses.json").read_text())["poses"]
    assert len(poses) == 30
    assert all(p["observed_corner_source"] == "solver_final_detection" for p in poses)


def test_unsupported_basis_records_failure_before_any_stub_fit(stubbed_calibration):
    session, args, _, calls = stubbed_calibration
    args.validation_basis = "hardware_exposure"
    with pytest.raises(ValueError, match="validation_basis"):
        solver.calibrate(args)
    assert calls == []
    report = json.loads(next(session.glob("solve-*/report.json")).read_text())
    assert report["status"] == "failed"
    assert report["validation_basis"] == "hardware_exposure"
