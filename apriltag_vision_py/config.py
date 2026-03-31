from dataclasses import dataclass
import json

@dataclass
class AppConfig:
    camera_index: int
    width: int
    height: int
    fps: int
    tag_family: str
    tag_size_m: float
    min_decision_margin: float
    draw_debug: bool
    publish_nt: bool
    team_number: int
    nt_table: str

def load_config(path: str) -> AppConfig:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return AppConfig(**data)

def load_calibration(path: str):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return data["camera_matrix"], data["dist_coeffs"]