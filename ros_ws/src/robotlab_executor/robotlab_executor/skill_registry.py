"""Skill registry loader and validator for RobotLab executor."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Tuple

import jsonschema
import yaml


@dataclass(frozen=True)
class Condition:
    """A single precondition to evaluate against robot state."""

    path: str
    op: str
    value: Any


def _get_path(d: Dict[str, Any], path: str) -> Any:
    """Navigate a nested dict using dot-separated path."""
    cur: Any = d
    for part in path.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return None
        cur = cur[part]
    return cur


def eval_condition(robot_state: Dict[str, Any], cond: Condition) -> bool:
    """Evaluate a single condition against robot state."""
    lhs = _get_path(robot_state, cond.path)
    op = cond.op
    rhs = cond.value

    if op == "==":
        return lhs == rhs
    if op == "!=":
        return lhs != rhs
    if op == ">":
        return lhs is not None and lhs > rhs
    if op == ">=":
        return lhs is not None and lhs >= rhs
    if op == "<":
        return lhs is not None and lhs < rhs
    if op == "<=":
        return lhs is not None and lhs <= rhs
    if op == "in":
        return lhs in rhs if isinstance(rhs, list) else False

    raise ValueError(f"Unknown operator: {op}")


def eval_preconditions(
    robot_state: Dict[str, Any], preconditions: Dict[str, Any]
) -> Tuple[bool, List[str]]:
    """
    Evaluate preconditions block against robot state.

    preconditions format:
      all:
        - {path, op, value}
      any:
        - {path, op, value}

    Returns (success, list_of_failure_messages).
    """
    failures: List[str] = []

    all_conds = preconditions.get("all", [])
    any_conds = preconditions.get("any", [])

    for raw in all_conds:
        c = Condition(**raw)
        if not eval_condition(robot_state, c):
            failures.append(
                f"FAILED(all): {c.path} {c.op} {c.value} "
                f"(got={_get_path(robot_state, c.path)})"
            )

    if any_conds:
        ok_any = False
        for raw in any_conds:
            c = Condition(**raw)
            if eval_condition(robot_state, c):
                ok_any = True
                break
        if not ok_any:
            failures.append("FAILED(any): none satisfied")

    return (len(failures) == 0, failures)


class SkillRegistry:
    """
    Loads and manages the skill registry YAML.

    Provides:
    - Skill existence checks
    - Argument validation against per-skill schemas
    - Precondition evaluation
    - Access to global safety limits
    """

    def __init__(self, yaml_path: Path) -> None:
        self.yaml_path = yaml_path
        self.data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))

        self.skills: Dict[str, Any] = self.data.get("skills", {})
        self.global_limits: Dict[str, Any] = self.data.get("global_safety_limits", {})

    def list_skills(self) -> List[str]:
        """Return sorted list of available skill names."""
        return sorted(self.skills.keys())

    def get_skill(self, name: str) -> Dict[str, Any]:
        """Get skill definition by name. Raises KeyError if not found."""
        if name not in self.skills:
            raise KeyError(f"Unknown skill: {name}")
        return self.skills[name]

    def validate_skill_call_args(self, skill_name: str, args: Dict[str, Any]) -> None:
        """
        Validate args against the skill's args_schema.
        Raises jsonschema.ValidationError on failure.
        """
        skill = self.get_skill(skill_name)
        schema = skill.get("args_schema", {"type": "object"})
        jsonschema.validate(instance=args, schema=schema)

    def check_preconditions(
        self, skill_name: str, robot_state: Dict[str, Any]
    ) -> Tuple[bool, List[str]]:
        """
        Check if preconditions are satisfied for the given skill.
        Returns (success, list_of_failure_messages).
        """
        skill = self.get_skill(skill_name)
        pre = skill.get("preconditions", {"all": []})
        return eval_preconditions(robot_state, pre)
