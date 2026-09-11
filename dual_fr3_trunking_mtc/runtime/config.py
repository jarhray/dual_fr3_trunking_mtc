from __future__ import annotations

from dataclasses import dataclass

from dual_fr3_moveit_config.backends import (
    DEFAULT_SIMULATION_BACKEND,
    SIMULATION_BACKENDS,
    resolve_simulation_backend,
)

from ..models import (
    DEFAULT_FOLLOWER_ORIENTATION_DIRECTION,
    DEFAULT_LEADER_LEAD_DISTANCE,
    DEFAULT_LEADER_ORIENTATION_DIRECTION,
    DEFAULT_TOOL_PITCH,
    DEFAULT_TOOL_ROLL,
)


@dataclass(frozen=True)
class TrunkingDefaults:
    """Single source of defaults shared by launch files and executable entrypoints."""

    # Robot environment.
    fake_sensor_commands: bool = True
    left_robot_ip: str = "172.16.0.2"
    right_robot_ip: str = "172.16.0.3"
    load_gripper: bool = True
    start_gripper: bool = True
    ee_id: str = "franka_hand"
    use_rviz: bool = True
    simulation_backend: str = DEFAULT_SIMULATION_BACKEND
    gz_args: str = "empty.sdf -r"
    gazebo_effort: bool = False

    # Task geometry and actor mapping.
    task_frame: str = "left_fr3_link0"
    initial_leader_index: int = 1
    initial_follower_index: int = 0
    leader_group: str = "left_fr3_arm"
    follower_group: str = "right_fr3_arm"
    leader_ik_frame: str = "left_fr3_hand_tcp"
    follower_ik_frame: str = "right_fr3_hand_tcp"
    leader_orientation_direction: str = DEFAULT_LEADER_ORIENTATION_DIRECTION
    follower_orientation_direction: str = DEFAULT_FOLLOWER_ORIENTATION_DIRECTION
    tool_roll: float = DEFAULT_TOOL_ROLL
    tool_pitch: float = DEFAULT_TOOL_PITCH
    leader_lead_distance: float = DEFAULT_LEADER_LEAD_DISTANCE

    # Motion planning.
    cartesian_step_size: float = 0.001
    # 1.5 truncates a continuous default FR3 path at about 90% because joint
    # increments naturally grow towards the endpoint. Keep jump detection on;
    # dense TCP validation independently rejects actual route excursions.
    cartesian_jump_threshold: float = 2.5
    cartesian_path_tolerance: float = 0.01
    motion_velocity_scaling: float = 0.2
    motion_acceleration_scaling: float = 0.2
    anchor_max_path_z: float = 0.5
    anchor_max_path_length_ratio: float = 1.5
    planning_attempts: int = 10
    execution_replan_attempts: int = 2
    trajectory_execution_duration_scaling: float = 10.0
    trajectory_execution_goal_margin: float = 5.0

    # Preparation and grippers.
    preparation_enabled: bool = True
    preparation_height: float = 0.05
    preparation_interactive: bool = True
    preparation_ik_candidates: int = 8
    preparation_ik_attempts: int = 80
    preparation_ik_timeout: float = 0.05
    preparation_min_joint_distance: float = 0.3
    preparation_candidate_attempts: int = 2
    preparation_search_timeout: float = 180.0
    preparation_leader_gripper_profile: str = "cable_tip"
    preparation_follower_gripper_profile: str = "cable_body"

    # Execution and interface lifecycle.
    plan: bool = True
    execute: bool = True
    execute_stage_by_stage: bool = True
    publish_solution: bool = True
    keep_alive_sec: float = 30.0

    # Startup readiness gate.
    readiness_timeout: float = 60.0
    state_max_age: float = 0.5
    home_grippers_before_execute: bool = True
    grippers_homed: bool = False

    # Planning-only visualization node.
    samples_per_segment: int = 8
    publish_markers: bool = True
    auto_reload: bool = True
    reload_period_sec: float = 1.0
    keypoint_marker_scale: float = 0.02
    segment_line_width: float = 0.015
    marker_z_offset: float = 0.0
    publish_labels: bool = True


DEFAULTS = TrunkingDefaults()


def launch_default(value: object) -> str:
    """Convert a typed default into the string form expected by ROS launch."""
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def parse_bool(value: str | bool) -> bool:
    """Parse the boolean spelling historically accepted by the CLI."""
    if isinstance(value, bool):
        return value
    return value.lower() in {"1", "true", "yes", "on"}
