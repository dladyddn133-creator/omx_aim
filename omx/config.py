"""Config 로딩 및 dataclass 변환."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import yaml


@dataclass
class MotorConfig:
    port: str
    profile_velocity: int
    profile_acceleration: int
    elbow_p_gain: int
    elbow_i_gain: int
    elbow_d_gain: int


@dataclass
class CalibrationConfig:
    home: dict[str, int]
    sign: dict[str, int]


@dataclass
class SafetyConfig:
    angle_limits_deg: dict[str, tuple[float, float]]
    max_step_deg: float
    large_delta_threshold_tick: int

    angle_limits_rad: dict[str, tuple[float, float]] = field(init=False)
    max_step_rad: float = field(init=False)

    def __post_init__(self):
        self.angle_limits_rad = {
            m: (math.radians(lo), math.radians(hi))
            for m, (lo, hi) in self.angle_limits_deg.items()
        }
        self.max_step_rad = math.radians(self.max_step_deg)


@dataclass
class KeyboardConfig:
    arm_step: int
    gripper_step: int
    command_interval: float


@dataclass
class IbvsConfig:
    camera_index: int
    kp_yaw: float
    kp_pitch: float
    sign_vs_x: int
    sign_vs_y: int
    deadband: float
    control_hz: float


@dataclass
class YoloConfig:
    model_path: str
    target_class: int
    conf_threshold: float
    imgsz: int


@dataclass
class FireConfig:
    hold_time_sec: float
    confirm_deadband_scale: float
    gripper_close_pos: int
    gripper_open_pos: int
    gripper_close_duration: float
    gripper_open_duration: float
    cooldown_sec: float


@dataclass
class AutoTrackConfig:
    default_armed: bool
    duplicate_threshold_m: float


@dataclass
class PatrolConfig:
    """정찰 + 우선순위 큐."""
    scan_timeout_sec: float
    max_queue_size: int
    duplicate_threshold_m: float


@dataclass
class Config:
    motor: MotorConfig
    calibration: CalibrationConfig
    safety: SafetyConfig
    keyboard: KeyboardConfig
    ibvs: IbvsConfig
    yolo: YoloConfig | None = None
    fire: FireConfig | None = None
    autotrack: AutoTrackConfig | None = None
    patrol: PatrolConfig | None = None


def find_config_path(path: str | None = None) -> Path:
    if path is not None:
        p = Path(path)
        if not p.is_file():
            raise FileNotFoundError(f"Config 파일 없음: {p}")
        return p

    here = Path(__file__).resolve()
    candidates = [
        Path.cwd() / "config.yaml",
        here.parent.parent / "config.yaml",
    ]
    for c in candidates:
        if c.is_file():
            return c

    raise FileNotFoundError(
        "config.yaml 을 찾을 수 없습니다:\n"
        + "\n".join(f"  - {c}" for c in candidates)
    )


def _tuple_pairs(d):
    out = {}
    for k, v in d.items():
        if len(v) != 2:
            raise ValueError(f"{k}: [lo, hi] 형태여야 하는데 {v}")
        out[k] = (float(v[0]), float(v[1]))
    return out


def load_config(path=None):
    config_path = find_config_path(path)
    with open(config_path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)

    try:
        yolo_cfg = YoloConfig(**raw["yolo"]) if "yolo" in raw else None
        fire_cfg = FireConfig(**raw["fire"]) if "fire" in raw else None
        autotrack_cfg = AutoTrackConfig(**raw["autotrack"]) if "autotrack" in raw else None
        patrol_cfg = PatrolConfig(**raw["patrol"]) if "patrol" in raw else None

        cfg = Config(
            motor=MotorConfig(**raw["motor"]),
            calibration=CalibrationConfig(**raw["calibration"]),
            safety=SafetyConfig(
                angle_limits_deg=_tuple_pairs(raw["safety"]["angle_limits_deg"]),
                max_step_deg=raw["safety"]["max_step_deg"],
                large_delta_threshold_tick=raw["safety"]["large_delta_threshold_tick"],
            ),
            keyboard=KeyboardConfig(**raw["keyboard"]),
            ibvs=IbvsConfig(**raw["ibvs"]),
            yolo=yolo_cfg,
            fire=fire_cfg,
            autotrack=autotrack_cfg,
            patrol=patrol_cfg,
        )
    except (KeyError, TypeError) as e:
        raise ValueError(
            f"Config 파일 형식 오류 ({config_path}): {e}"
        ) from e

    return cfg