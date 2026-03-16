"""YAML adapter for yaw_rate_steering_controller runtime options."""

from __future__ import annotations

from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any, Dict, Mapping, Optional

import yaml

from vehicle_sim.controllers.yaw_rate_steering_controller.controller import YawRateSteeringControllerOptions


_ALLOWED_MODES = {"ff", "ff_fb", "ff_fb_ls"}

_DEFAULT_MODE_MAPPING: Dict[str, Dict[str, Any]] = {
    "ff": {
        "enable_yaw_feedback": False,
        "enable_fy_feedback": False,
        "enable_steer_feedback": False,
        "enable_estimator": False,
    },
    "ff_fb": {
        "enable_yaw_feedback": True,
        "enable_fy_feedback": True,
        "fy_feedback_source": "estimate",
        "use_lateral_force_estimator": True,
        "enable_steer_feedback": True,
        "enable_estimator": False,
    },
    "ff_fb_ls": {
        "enable_yaw_feedback": True,
        "enable_fy_feedback": True,
        "fy_feedback_source": "estimate",
        "use_lateral_force_estimator": True,
        "use_slip_angle_estimator": True,
        "enable_steer_feedback": True,
        "enable_estimator": True,
        "enable_b_estimator": True,
        "enable_c_alpha_estimator": True,
    },
}


@dataclass
class ControllerRuntimeConfig:
    mode: str
    options: YawRateSteeringControllerOptions
    vehicle_config_path: Optional[str]
    gains_path: Optional[str]
    dt_explicit: bool


def load_controller_runtime_config(config_path: str | Path) -> ControllerRuntimeConfig:
    """Load controller mode/options/runtime paths from YAML config."""
    path = Path(config_path)
    with path.open("r", encoding="utf-8-sig") as f:
        raw = yaml.safe_load(f) or {}

    if not isinstance(raw, Mapping):
        raise ValueError("controller config root must be a mapping")

    controller_cfg = _as_mapping(raw.get("controller", {}), "controller")
    mode_mapping_cfg = _as_mapping(raw.get("mode_mapping", {}), "mode_mapping")
    advanced_cfg = _as_mapping(raw.get("advanced", {}), "advanced")

    mode = str(controller_cfg.get("mode", "ff_fb_ls")).strip().lower()
    if mode not in _ALLOWED_MODES:
        raise ValueError(f"controller.mode must be one of: {sorted(_ALLOWED_MODES)}")

    option_field_names = {f.name for f in fields(YawRateSteeringControllerOptions)}
    direct_option_keys = {
        str(k)
        for k in controller_cfg.keys()
        if str(k) not in {"mode", "vehicle_config_path", "gains_path"}
    }
    uses_flat_controller_options = bool(
        direct_option_keys & (option_field_names | {"b_estimator", "c_alpha_estimator"})
    )

    mode_mapping = dict(_DEFAULT_MODE_MAPPING)
    for key, value in mode_mapping_cfg.items():
        key_norm = str(key).strip().lower()
        if key_norm not in _ALLOWED_MODES:
            continue
        mode_mapping[key_norm] = _as_mapping(value, f"mode_mapping.{key_norm}")

    resolved: Dict[str, Any] = dict(mode_mapping[mode])
    if uses_flat_controller_options:
        resolved.update(
            {
                str(k): v
                for k, v in controller_cfg.items()
                if str(k) not in {"mode", "vehicle_config_path", "gains_path"}
            }
        )
    else:
        advanced_enabled = bool(advanced_cfg.get("enabled", False))
        if advanced_enabled:
            override_raw = advanced_cfg.get(
                "controller_options_override",
                advanced_cfg.get("block_controller_options_override", {}),
            )
            override = _as_mapping(
                override_raw,
                "advanced.controller_options_override",
            )
            resolved.update(override)

    resolved = _expand_specialized_estimator_options(resolved)

    dt_explicit = "dt" in resolved

    allowed_fields = option_field_names
    unknown_keys = sorted(k for k in resolved.keys() if k not in allowed_fields)
    if unknown_keys:
        raise ValueError(f"unknown YawRateSteeringControllerOptions keys: {unknown_keys}")

    options = YawRateSteeringControllerOptions(**resolved)

    vehicle_config_path = _normalize_nullable_path(controller_cfg.get("vehicle_config_path", None))
    gains_path = _normalize_nullable_path(controller_cfg.get("gains_path", None))

    return ControllerRuntimeConfig(
        mode=mode,
        options=options,
        vehicle_config_path=vehicle_config_path,
        gains_path=gains_path,
        dt_explicit=dt_explicit,
    )


def _as_mapping(value: Any, key_name: str) -> Mapping[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise ValueError(f"{key_name} must be a mapping")
    return value


def _normalize_nullable_path(value: Any) -> Optional[str]:
    if value is None:
        return None
    text = str(value).strip()
    if text == "" or text.lower() == "null":
        return None
    return text


def _expand_specialized_estimator_options(resolved: Mapping[str, Any]) -> Dict[str, Any]:
    out = dict(resolved)
    out.update(_remap_estimator_block(out.pop("b_estimator", {}), "b_estimator", "b_estimator"))
    out.update(
        _remap_estimator_block(
            out.pop("c_alpha_estimator", {}),
            "c_alpha_estimator",
            "c_alpha_estimator",
        )
    )
    return out


def _remap_estimator_block(value: Any, key_name: str, prefix: str) -> Dict[str, Any]:
    block = _as_mapping(value, f"controller.{key_name}")
    if not block:
        return {}

    allowed = {
        "lambda",
        "forgetting_factor",
        "p0",
        "min_value",
        "max_value",
        "start_time",
        "min_samples",
        "sample_decimation",
        "dot_min_abs",
        "angle_margin_rad",
        "rate_margin_rad_s",
        "alpha_min_abs",
        "fz_min",
        "fy_saturation_margin",
        "use_torque_fallback",
    }
    unknown_keys = sorted(str(k) for k in block.keys() if str(k) not in allowed)
    if unknown_keys:
        raise ValueError(
            f"unknown keys in controller.{key_name}: {unknown_keys}"
        )

    remapped: Dict[str, Any] = {}
    if "lambda" in block:
        remapped[f"{prefix}_forgetting_factor"] = block["lambda"]
    if "forgetting_factor" in block:
        remapped[f"{prefix}_forgetting_factor"] = block["forgetting_factor"]
    if "p0" in block:
        remapped[f"{prefix}_p0"] = block["p0"]
    if "min_value" in block:
        remapped[f"{prefix}_min_value"] = block["min_value"]
    if "max_value" in block:
        remapped[f"{prefix}_max_value"] = block["max_value"]
    if "start_time" in block:
        remapped[f"{prefix}_start_time"] = block["start_time"]
    if "min_samples" in block:
        remapped[f"{prefix}_min_samples"] = block["min_samples"]
    if "sample_decimation" in block:
        remapped[f"{prefix}_sample_decimation"] = block["sample_decimation"]
    if "dot_min_abs" in block:
        remapped[f"{prefix}_dot_min_abs"] = block["dot_min_abs"]
    if "angle_margin_rad" in block:
        remapped[f"{prefix}_angle_margin_rad"] = block["angle_margin_rad"]
    if "rate_margin_rad_s" in block:
        remapped[f"{prefix}_rate_margin_rad_s"] = block["rate_margin_rad_s"]
    if "alpha_min_abs" in block:
        remapped[f"{prefix}_alpha_min_abs"] = block["alpha_min_abs"]
    if "fz_min" in block:
        remapped[f"{prefix}_fz_min"] = block["fz_min"]
    if "fy_saturation_margin" in block:
        remapped[f"{prefix}_fy_saturation_margin"] = block["fy_saturation_margin"]
    if "use_torque_fallback" in block:
        remapped[f"{prefix}_use_torque_fallback"] = block["use_torque_fallback"]
    return remapped
