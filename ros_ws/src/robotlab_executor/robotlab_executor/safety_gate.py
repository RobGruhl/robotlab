"""Safety gate for enforcing global limits on skill execution."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Tuple


@dataclass(frozen=True)
class GateResult:
    """Result of safety gate evaluation."""

    ok: bool
    reason: str
    merged_constraints: Dict[str, Any]


def merge_constraints(
    defaults: Dict[str, Any], overrides: Dict[str, Any]
) -> Dict[str, Any]:
    """Merge default constraints with overrides."""
    merged = dict(defaults or {})
    for k, v in (overrides or {}).items():
        merged[k] = v
    return merged


def clamp_numeric(value: Any, min_v: float, max_v: float) -> Tuple[bool, Any]:
    """
    Clamp a numeric value to [min_v, max_v].
    Returns (was_clamped, clamped_value).
    """
    if not isinstance(value, (int, float)):
        return False, value
    if value < min_v:
        return True, min_v
    if value > max_v:
        return True, max_v
    return False, value


def enforce_global_limits(
    skill_name: str, constraints: Dict[str, Any], global_limits: Dict[str, Any]
) -> GateResult:
    """
    Enforce global safety limits on skill constraints.

    Clamps known speed/altitude values to global maximums.
    Extend this aggressively over time for new constraint types.
    """
    merged = dict(constraints or {})

    if skill_name.startswith("base."):
        limits = global_limits.get("base", {})
        for k in ["max_linear_speed_m_s", "max_angular_speed_rad_s"]:
            if k in merged and k in limits:
                changed, merged[k] = clamp_numeric(merged[k], 0.0, float(limits[k]))
                if changed:
                    return GateResult(
                        ok=True,
                        reason=f"Clamped {k} to global limit {limits[k]}",
                        merged_constraints=merged,
                    )

    if skill_name.startswith("drone."):
        limits = global_limits.get("drone", {})
        for k, limit_key in [
            ("max_horizontal_speed_m_s", "max_horizontal_speed_m_s"),
            ("max_vertical_speed_m_s", "max_vertical_speed_m_s"),
            ("max_altitude_m", "max_altitude_m"),
        ]:
            if k in merged and limit_key in limits:
                changed, merged[k] = clamp_numeric(
                    merged[k], 0.0, float(limits[limit_key])
                )
                if changed:
                    return GateResult(
                        ok=True,
                        reason=f"Clamped {k} to global limit {limits[limit_key]}",
                        merged_constraints=merged,
                    )

    if skill_name.startswith("arm."):
        limits = global_limits.get("arm", {})
        for k in ["max_joint_speed_rad_s", "max_cartesian_speed_m_s"]:
            if k in merged and k in limits:
                changed, merged[k] = clamp_numeric(merged[k], 0.0, float(limits[k]))
                if changed:
                    return GateResult(
                        ok=True,
                        reason=f"Clamped {k} to global limit {limits[k]}",
                        merged_constraints=merged,
                    )

    return GateResult(ok=True, reason="OK", merged_constraints=merged)
