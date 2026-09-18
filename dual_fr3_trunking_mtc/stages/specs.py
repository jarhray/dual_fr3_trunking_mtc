from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any, Dict, Sequence

from dual_fr3_trunking_mtc.task.models import DEFAULT_LEADER_ORIENTATION_DIRECTION


@dataclass(frozen=True)
class MtcStageSpec:
    """Serializable description of one preparation or formal MTC stage."""

    step_index: int
    stage_index: int
    stage_key: str
    action: str
    actor: str
    group: str
    ik_frame: str
    name: str
    from_index: int
    to_index: int
    from_keypoint: str
    to_keypoint: str
    frame_id: str
    vector: tuple[float, float, float]
    execution_order: list[str]
    primitive: str
    mtc_stage_type: str
    planner: str
    executable: bool = True
    yaw_delta: float = 0.0
    target_yaw: float = 0.0
    joint_goal: Dict[str, float] = field(default_factory=dict)
    gripper_profile: str = ""
    gripper_action: str = ""
    gripper_width_override: float | None = None
    cable_config: str = ""
    cable_operation: str = "spawn"
    cable_orientation_direction: str = DEFAULT_LEADER_ORIENTATION_DIRECTION
    cable_preparation_poses: Dict[str, Any] = field(default_factory=dict)
    phase: str = "formal"
    confirmation_required: bool = False
    children: tuple[MtcStageSpec, ...] = ()
    info: str = ""


def mtc_stage_spec_to_dict(spec: MtcStageSpec) -> Dict[str, Any]:
    return {
        "stage_index": spec.stage_index,
        "stage_key": spec.stage_key,
        "task_step_index": spec.step_index,
        "action": spec.action,
        "actor": spec.actor,
        "group": spec.group,
        "ik_frame": spec.ik_frame,
        "stage_name": spec.name,
        "primitive": spec.primitive,
        "from_index": spec.from_index,
        "to_index": spec.to_index,
        "from": spec.from_keypoint,
        "to": spec.to_keypoint,
        "frame_id": spec.frame_id,
        "relative_vector": list(spec.vector),
        "yaw_delta": spec.yaw_delta,
        "target_yaw": spec.target_yaw,
        "joint_goal": dict(spec.joint_goal),
        "gripper_profile": spec.gripper_profile,
        "gripper_action": spec.gripper_action,
        "gripper_width_override": spec.gripper_width_override,
        "cable_config": spec.cable_config,
        "cable_operation": spec.cable_operation,
        "cable_orientation_direction": spec.cable_orientation_direction,
        "cable_preparation_poses": spec.cable_preparation_poses,
        "phase": spec.phase,
        "confirmation_required": spec.confirmation_required,
        "children": [mtc_stage_spec_to_dict(child) for child in spec.children],
        "execution_order": list(spec.execution_order),
        "mtc_stage_type": spec.mtc_stage_type,
        "planner": spec.planner,
        "executable": spec.executable,
        "info": spec.info,
    }


def mtc_stage_sequence_to_dict(
    specs: Sequence[MtcStageSpec],
    task_name: str = "dual_fr3_trunking_mtc_prototype",
) -> Dict[str, Any]:
    return {
        "interface_version": 2,
        "task_name": task_name,
        "stage_count": len(specs),
        "stage_sequence": [mtc_stage_spec_to_dict(spec) for spec in specs],
    }


def mtc_stage_sequence_to_json(specs: Sequence[MtcStageSpec]) -> str:
    return json.dumps(
        mtc_stage_sequence_to_dict(specs),
        ensure_ascii=False,
        indent=2,
    )


def mtc_stage_sequence_to_text(specs: Sequence[MtcStageSpec]) -> str:
    lines = []
    for spec in specs:
        execution_order = " -> ".join(spec.execution_order)
        lines.append(
            f"{spec.stage_index}: stage_key={spec.stage_key}, "
            f"task_step={spec.step_index}, "
            f"action={spec.action}, actor={spec.actor}, "
            f"primitive={spec.primitive}, type={spec.mtc_stage_type}, "
            f"group={spec.group}, ik_frame={spec.ik_frame}, "
            f"{spec.from_keypoint}->{spec.to_keypoint}, "
            f"frame={spec.frame_id}, vector={spec.vector}, "
            f"yaw_delta={spec.yaw_delta:.3f}, "
            f"target_yaw={spec.target_yaw:.3f}, "
            f"gripper_profile={spec.gripper_profile or '-'}, "
            f"gripper_action={spec.gripper_action or '-'}, "
            f"gripper_width_override={spec.gripper_width_override}, "
            f"phase={spec.phase}, "
            f"confirmation_required={spec.confirmation_required}, "
            f"children={len(spec.children)}, "
            f"executable={spec.executable}, "
            f"order={execution_order}, info={spec.info}"
        )
    return "\n".join(lines)
