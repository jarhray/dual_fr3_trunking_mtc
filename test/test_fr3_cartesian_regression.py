"""Reproduce the 90% cutoff with the actual dual FR3 model and collision geometry."""

import copy
import logging
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml
import xacro

from dual_fr3_trunking_mtc.models import Keypoint, rpy_to_quaternion
from dual_fr3_trunking_mtc.mtc.preparation_search import pose_in_model
from dual_fr3_trunking_mtc.mtc.task_builder import create_mtc_task
from dual_fr3_trunking_mtc.planner import build_segment_plans
from dual_fr3_trunking_mtc.runtime.config import DEFAULTS
from dual_fr3_trunking_mtc.scheduler import build_task_plan
from dual_fr3_trunking_mtc.stages.compiler import build_mtc_stage_specs


def test_fr3_continuous_line_passes_without_disabling_jump_or_tcp_checks(tmp_path, caplog):
    import rclcpp
    from geometry_msgs.msg import PoseStamped
    from moveit.core.planning_scene import PlanningScene
    from moveit.task_constructor import core

    caplog.set_level(logging.INFO)
    config = Path(get_package_share_directory("dual_fr3_moveit_config")) / "config"
    parameters = {
        "robot_description": xacro.process_file(
            str(config / "dual_fr3.gazebo.urdf.xacro"),
        ).toxml(),
        "robot_description_semantic": xacro.process_file(
            str(config / "dual_fr3.srdf.xacro"),
        ).toxml(),
        **yaml.safe_load((config / "kinematics.yaml").read_text()),
    }
    parameter_file = tmp_path / "fr3.yaml"
    parameter_file.write_text(yaml.safe_dump({"/**": {"ros__parameters": parameters}}))
    rclcpp.init()
    try:
        node = rclcpp.Node("fr3_cartesian_regression", rclcpp.NodeOptions(
            automatically_declare_parameters_from_overrides=True,
            arguments=["--ros-args", "--params-file", str(parameter_file)],
        ))
        owner = core.Task(introspection=False)
        owner.loadRobotModel(node)
        scene = PlanningScene(owner.getRobotModel())
        state = copy.copy(scene.current_state)
        state.set_to_default_values("dual_fr3_arms", "both_ready")
        state.joint_positions = {
            "left_fr3_finger_joint1": 0.0025, "right_fr3_finger_joint1": 0.0025,
        }
        state.update()
        scene.current_state = state
        frame = "left_fr3_link0"
        target = PoseStamped()
        target.header.frame_id = frame
        target.pose.position.x, target.pose.position.y, target.pose.position.z = (
            0.604, 0.733, 0.05,
        )
        target.pose.orientation = rpy_to_quaternion(math.pi, 0.0, -math.pi / 2)
        assert state.set_from_ik(
            "right_fr3_arm", pose_in_model(scene, target), "right_fr3_hand_tcp", 0.05,
        )
        state.update()
        assert scene.is_state_valid(state, "right_fr3_arm")
        scene.current_state = state
        plan = build_task_plan(build_segment_plans([
            Keypoint("corner_1", frame, (0.604, 0.733, 0.05), True, "pull"),
            Keypoint("corner_2", frame, (0.604, 0.133, 0.05), True, "pull"),
            Keypoint("corner_4", frame, (0.364, 0.133, 0.05), True, "turn"),
            Keypoint("entry_5", frame, (0.364, -0.07, 0.10), False, "seat_edge"),
        ]))
        specs = tuple(s for s in build_mtc_stage_specs(plan)
                      if s.executable and s.actor == "follower")
        for threshold, accepted in [(1.5, False), (DEFAULTS.cartesian_jump_threshold, True)]:
            task, _ = create_mtc_task(
                node, plan, specs, start_scene=scene,
                cartesian_step_size=DEFAULTS.cartesian_step_size,
                cartesian_jump_threshold=threshold,
                cartesian_path_tolerance=DEFAULTS.cartesian_path_tolerance,
            )
            assert bool(task.plan(1)) is accepted
            if accepted:
                assert all(task[s.name].solutions for s in specs)
                assert "Cartesian TCP in-place deviation" in caplog.text
                assert "accepted" in caplog.text
            else:
                assert "Achieved: 0.901639" in task[specs[0].name].failures[0].comment
    finally:
        rclcpp.shutdown()
