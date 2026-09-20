"""Runtime configuration persistence, upload validation, and HTTP reload handoff."""
import copy
import json
from pathlib import Path
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest
import yaml

from custom_vision.control import RuntimeController
from custom_vision.dashboard import Dashboard


def config():
    return {"pipelines": [{"name": "front_tags", "type": "apriltag", "enabled": True,
                           "camera": {"source": 0, "width": 640, "height": 480, "fps": 30},
                           "settings": {"backend": "pupil", "mode": "2d"}, "robot_to_camera": None}],
            "dashboard": {"enabled": False, "port": 5801}, "networktables": {"enabled": False}}


def calibration():
    return {"width": 640, "height": 480, "camera_matrix": [[500, 0, 320], [0, 500, 240], [0, 0, 1]],
            "dist_coeffs": [0, 0, 0, 0, 0]}


def layout():
    return {"field": {"length": 16.54, "width": 8.02}, "tags": [
        {"ID": 7, "pose": {"translation": {"x": 3., "y": 2., "z": 1.},
                            "rotation": {"quaternion": {"W": 1., "X": 0., "Y": 0., "Z": 0.}}}}]}


@pytest.fixture
def controller(tmp_path):
    path = tmp_path / "config" / "vision.yaml"
    path.parent.mkdir()
    path.write_text(yaml.safe_dump(config()))
    restarted = threading.Event()
    return RuntimeController(path, restarted.set), restarted


def test_config_apply_is_atomic_and_preserves_input(controller):
    instance, restarted = controller
    original = instance.get_config()
    submitted = copy.deepcopy(original)
    submitted["pipelines"][0]["settings"]["threads"] = 3
    submitted["pipelines"][0]["calibration_data"] = {"discard": True}
    submitted["field_layout_data"] = {"discard": True}
    snapshot = copy.deepcopy(submitted)
    validations = []
    instance.validate_runtime = validations.append
    result = instance.apply_config(submitted)
    assert result["saved"] and result["restart_required"]
    assert submitted == snapshot
    saved = instance.get_config()
    assert saved["pipelines"][0]["settings"]["threads"] == 3
    assert "calibration_data" not in saved["pipelines"][0]
    assert "field_layout_data" not in saved
    assert saved["pipelines"][0]["robot_to_camera"] is None
    assert len(validations) == 1
    assert not list(instance.path.parent.glob(".pending-*"))
    assert restarted.wait(1)


def test_invalid_config_and_backend_initialization_do_not_replace_file(controller):
    instance, restarted = controller
    original_bytes = instance.path.read_bytes()
    bad = instance.get_config()
    bad["pipelines"][0]["settings"]["threads"] = 0
    with pytest.raises(ValueError):
        instance.apply_config(bad)
    assert instance.path.read_bytes() == original_bytes
    def reject(_):
        raise RuntimeError("Backend unavailable")
    instance.validate_runtime = reject
    with pytest.raises(RuntimeError, match="Backend unavailable"):
        instance.apply_config(instance.get_config())
    assert instance.path.read_bytes() == original_bytes
    assert not restarted.is_set()


def test_calibration_upload_is_validated_before_artifact_or_config_write(controller):
    instance, restarted = controller
    original_bytes = instance.path.read_bytes()
    bad = calibration(); bad["camera_matrix"][0][0] = -500
    with pytest.raises(ValueError):
        instance.upload_calibration("front_tags", bad)
    with pytest.raises(ValueError, match="Unknown pipeline"):
        instance.upload_calibration("missing", calibration())
    assert instance.path.read_bytes() == original_bytes
    assert not (instance.path.parent.parent / "calibration").exists()
    assert not restarted.is_set()


def test_valid_calibration_is_content_addressed_and_previous_version_survives(controller):
    instance, restarted = controller
    first = calibration()
    instance.upload_calibration("front_tags", first)
    first_path = (instance.path.parent / instance.get_config()["pipelines"][0]["calibration"]).resolve()
    assert first_path.parent == instance.path.parent.parent / "calibration"
    assert json.loads(first_path.read_text())["camera_matrix"] == first["camera_matrix"]
    instance.upload_calibration("front_tags", first)
    assert (instance.path.parent / instance.get_config()["pipelines"][0]["calibration"]).resolve() == first_path
    second = calibration(); second["camera_matrix"][0][0] = 510
    instance.upload_calibration("front_tags", second)
    second_path = (instance.path.parent / instance.get_config()["pipelines"][0]["calibration"]).resolve()
    assert second_path != first_path and first_path.exists()
    assert json.loads(first_path.read_text())["camera_matrix"][0][0] == 500
    assert restarted.wait(1)


def verified_poi_config(instance):
    """Create an existing, explicitly acknowledged calibration without a restart."""
    data=instance.get_config()
    selected=data['pipelines'][0]
    selected['calibration']='original-calibration.json'
    selected['poi']={'enabled':True,'calibration_verified':True,
                     'targets':[{'name':'aim','tag_id':7,'offset_m':[0,0,.2]}]}
    (instance.path.parent/selected['calibration']).write_text(json.dumps(calibration()))
    instance.path.write_text(yaml.safe_dump(data))
    return data


def test_identical_calibration_upload_preserves_verification_and_artifact(controller):
    instance,_=controller
    before=verified_poi_config(instance)
    # JSON ordering/formatting does not change the calibration's identity.
    reordered=dict(reversed(list(calibration().items())))
    result=instance.upload_calibration('front_tags',reordered)
    saved=instance.get_config()['pipelines'][0]
    assert saved['calibration']==before['pipelines'][0]['calibration']
    assert saved['poi']['calibration_verified'] is True
    assert result['poi_verification_reset']==[]
    assert not (instance.path.parent.parent/'calibration').exists()


def test_new_calibration_upload_clears_old_acknowledgement_then_allows_reverification(controller):
    instance,_=controller
    before=verified_poi_config(instance)
    replacement=calibration()
    replacement['camera_matrix'][0][0]=515
    result=instance.upload_calibration('front_tags',replacement)
    saved=instance.get_config()
    assert saved['pipelines'][0]['calibration']!=before['pipelines'][0]['calibration']
    assert saved['pipelines'][0]['poi']['calibration_verified'] is False
    assert result['poi_verification_reset']==['front_tags']
    assert 'verification cleared' in result['message']
    # A later, explicit acknowledgement of the selected calibration is retained.
    saved['pipelines'][0]['poi']['calibration_verified']=True
    instance.apply_config(saved)
    result=instance.upload_calibration('front_tags',replacement)
    assert instance.get_config()['pipelines'][0]['poi']['calibration_verified'] is True
    assert result['poi_verification_reset']==[]


@pytest.mark.parametrize('replacement_kind',['new_path','removed'])
def test_raw_config_calibration_change_resets_verification_before_backend_validation(controller,replacement_kind):
    instance,_=controller
    submitted=verified_poi_config(instance)
    if replacement_kind=='new_path':
        submitted['pipelines'][0]['calibration']='another-calibration.json'
        (instance.path.parent/'another-calibration.json').write_text(json.dumps(calibration()))
    else:
        submitted['pipelines'][0]['calibration']=None
    validations=[]
    instance.validate_runtime=validations.append
    result=instance.apply_config(submitted)
    assert result['poi_verification_reset']==['front_tags']
    assert instance.get_config()['pipelines'][0]['poi']['calibration_verified'] is False
    assert validations[0]['pipelines'][0]['poi']['calibration_verified'] is False
    assert submitted['pipelines'][0]['poi']['calibration_verified'] is True


def test_equivalent_calibration_path_keeps_current_acknowledgement(controller):
    instance,_=controller
    submitted=verified_poi_config(instance)
    submitted['pipelines'][0]['calibration']=str((instance.path.parent/'original-calibration.json').resolve())
    result=instance.apply_config(submitted)
    assert result['poi_verification_reset']==[]
    assert instance.get_config()['pipelines'][0]['poi']['calibration_verified'] is True


@pytest.mark.parametrize('route',['upload','raw_config'])
def test_rejected_calibration_change_keeps_old_file_and_acknowledgement(controller,route):
    instance,restarted=controller
    submitted=verified_poi_config(instance)
    original=instance.path.read_bytes()
    validations=[]
    def reject(config):
        validations.append(config)
        raise RuntimeError('Backend unavailable')
    instance.validate_runtime=reject
    replacement=calibration()
    replacement['camera_matrix'][0][0]=515
    with pytest.raises(RuntimeError,match='Backend unavailable'):
        if route=='upload':
            instance.upload_calibration('front_tags',replacement)
        else:
            (instance.path.parent/'replacement.json').write_text(json.dumps(replacement))
            submitted['pipelines'][0]['calibration']='replacement.json'
            instance.apply_config(submitted)
    assert validations[0]['pipelines'][0]['poi']['calibration_verified'] is False
    assert instance.path.read_bytes()==original
    assert instance.get_config()['pipelines'][0]['poi']['calibration_verified'] is True
    assert not list(instance.path.parent.glob('.pending-*'))
    assert not restarted.is_set()


def test_failed_atomic_calibration_save_keeps_old_acknowledgement(controller,monkeypatch):
    from custom_vision import control
    instance,restarted=controller
    verified_poi_config(instance)
    original=instance.path.read_bytes()
    replace=control.os.replace
    def reject_config_write(source,destination):
        if Path(destination)==instance.path:
            raise OSError('config replacement failed')
        return replace(source,destination)
    monkeypatch.setattr(control.os,'replace',reject_config_write)
    replacement=calibration()
    replacement['camera_matrix'][0][0]=515
    with pytest.raises(OSError,match='config replacement failed'):
        instance.upload_calibration('front_tags',replacement)
    assert instance.path.read_bytes()==original
    assert instance.get_config()['pipelines'][0]['poi']['calibration_verified'] is True
    assert not list(instance.path.parent.glob('.pending-*'))
    assert not restarted.is_set()


def test_field_upload_requires_complete_wpilib_layout_and_no_duplicate_ids(controller):
    instance, restarted = controller
    original = instance.path.read_bytes()
    for invalid in ({"tags": []}, {"field": {"length": 1, "width": 1}, "tags": []},
                    {**layout(), "tags": layout()["tags"] * 2}):
        with pytest.raises(ValueError):
            instance.upload_field_layout(invalid)
    assert instance.path.read_bytes() == original
    assert not restarted.is_set()
    instance.upload_field_layout(layout())
    saved = instance.get_config()
    assert json.loads((instance.path.parent / saved["field_layout"]).read_text()) == layout()
    assert restarted.wait(1)


def test_http_save_returns_before_reload_and_subsequent_server_reads_saved_config(controller):
    instance, restarted = controller
    dashboard = Dashboard({"host": "127.0.0.1", "port": 0}, instance)
    address = f"http://127.0.0.1:{dashboard.server.server_port}"
    try:
        with urlopen(address + "/api/config", timeout=2) as response:
            setup = json.load(response)
        updated = setup["config"]
        updated["pipelines"][0]["preview"] = {"rotation_deg": 270}
        request = Request(address + "/api/config", data=json.dumps(updated).encode(), method="POST",
                          headers={"Content-Type": "application/json", "X-Custom-Vision-CSRF": setup["csrf_token"], "Origin": address})
        with urlopen(request, timeout=2) as response:
            result = json.load(response)
        assert result["saved"]
        assert restarted.wait(1)
    finally:
        dashboard.close()
    # This exercises the HTTP/controller handoff without opening real cameras or
    # claiming a physical capture restart has been validated.
    replacement = Dashboard({"host": "127.0.0.1", "port": 0}, instance)
    try:
        with urlopen(f"http://127.0.0.1:{replacement.server.server_port}/api/config", timeout=2) as response:
            setup = json.load(response)
        assert setup["config"]["pipelines"][0]["preview"]["rotation_deg"] == 270
    finally:
        replacement.close()
