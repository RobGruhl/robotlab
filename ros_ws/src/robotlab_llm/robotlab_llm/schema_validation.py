"""Schema validation utilities for RobotLab LLM outputs."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict

import jsonschema


@dataclass(frozen=True)
class SchemaBundle:
    """Container for all RobotLab JSON schemas."""

    scene_analysis: Dict[str, Any]
    skill_call: Dict[str, Any]
    action_plan: Dict[str, Any]


def load_schemas(schema_dir: Path) -> SchemaBundle:
    """Load all schemas from the given directory."""

    def _load(name: str) -> Dict[str, Any]:
        return json.loads((schema_dir / name).read_text(encoding="utf-8"))

    return SchemaBundle(
        scene_analysis=_load("scene_analysis.schema.json"),
        skill_call=_load("skill_call.schema.json"),
        action_plan=_load("action_plan.schema.json"),
    )


class SchemaValidator:
    """Validates LLM/VLM outputs against RobotLab JSON schemas."""

    def __init__(self, schema_dir: Path) -> None:
        self.schema_dir = schema_dir
        self.schemas = load_schemas(schema_dir)

        # Resolver for local $ref files (action_plan -> skill_call).
        self._resolver = jsonschema.RefResolver(
            base_uri=schema_dir.as_uri() + "/",
            referrer=self.schemas.action_plan,
        )

    def validate_scene_analysis(self, obj: Dict[str, Any]) -> None:
        """Validate a SceneAnalysis object. Raises ValidationError on failure."""
        jsonschema.validate(instance=obj, schema=self.schemas.scene_analysis)

    def validate_skill_call(self, obj: Dict[str, Any]) -> None:
        """Validate a SkillCall object. Raises ValidationError on failure."""
        jsonschema.validate(instance=obj, schema=self.schemas.skill_call)

    def validate_action_plan(self, obj: Dict[str, Any]) -> None:
        """Validate an ActionPlan object. Raises ValidationError on failure."""
        jsonschema.validate(
            instance=obj,
            schema=self.schemas.action_plan,
            resolver=self._resolver,
        )
