from pathlib import Path
import logging
from types import SimpleNamespace as NS

import pytest

from dual_fr3_trunking_mtc.planner import load_keypoints
from dual_fr3_trunking_mtc.planner import build_segment_plans
from dual_fr3_trunking_mtc.scheduler import build_task_plan
from dual_fr3_trunking_mtc.preparation import PreparationConfig, build_preparation_stage_specs
from dual_fr3_trunking_mtc.simulation_cable import preparation_cable_config


ROOT = Path(__file__).resolve().parents[2]
CONFIG = str(ROOT / "dual_fr3_maniskill/config/trunking_cable.yaml")


@pytest.fixture(autouse=True)
def preserve_python_logger_class():
    # The native model test lazily imports launch_ros. ROS launch installs a
    # global logger class; keep that side effect out of later task unit tests.
    previous = logging.getLoggerClass()
    yield
    logging.setLoggerClass(previous)


@pytest.mark.parametrize("backend", ["fake", "gazebo", "real"])
def test_other_backends_do_not_even_load_cable_config(backend):
    assert preparation_cable_config(backend, True, True, "/missing.yaml") == ""


def test_maniskill_requires_preparation_or_explicit_cable_disable():
    assert preparation_cable_config("maniskill", False, False, "") == ""
    with pytest.raises(ValueError, match="requires preparation"):
        preparation_cable_config("maniskill", True, False, "")


def cable_specs():
    keypoints = load_keypoints(ROOT / "dual_fr3_trunking_mtc/config/keypoints.yaml")
    plan = build_task_plan(build_segment_plans(keypoints), initial_leader_index=1, initial_follower_index=0)
    return build_preparation_stage_specs(plan, PreparationConfig(simulation_cable_config=CONFIG))


def test_cable_stage_is_after_successful_closures_and_before_descent():
    specs = cable_specs()
    assert [s.mtc_stage_type for s in specs] == [
        "MoveTo", "MoveTo", "GripperOperation", "GripperOperation", "SimulationCable", "Merger"]
    assert specs[2].gripper_width_override == pytest.approx(.0074)
    assert specs[3].gripper_width_override == 0.
    assert specs[2].gripper_action == specs[3].gripper_action == "move"
    assert [s.stage_index for s in specs] == list(range(6))
    assert all(s.stage_index == 5 for s in specs[-1].children)


def test_planning_scene_attaches_usb_in_the_tcp_heading():
    from dual_fr3_maniskill.cable.planning_scene import attached_usb_scene
    from dual_fr3_maniskill.cable.model import USB_LINK
    from scipy.spatial.transform import Rotation
    import numpy as np
    scene = attached_usb_scene(CONFIG)
    attachment, = scene.robot_state.attached_collision_objects
    assert attachment.link_name == attachment.object.header.frame_id == "left_fr3_hand_tcp"
    assert attachment.object.id == USB_LINK
    assert "right_fr3_hand_tcp" not in attachment.touch_links
    q = attachment.object.mesh_poses[0].orientation
    rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    np.testing.assert_allclose(rotation @ [0., 1., 0.], [1., 0., 0.], atol=1e-10)
    assert scene.is_diff and scene.robot_state.is_diff


@pytest.mark.parametrize("closed,reserved,active,success", [
    (set(), set(), False, False), ({"left"}, set(), False, False),
    ({"left", "right"}, {("right", "gripper")}, False, False),
    ({"left", "right"}, set(), False, True),
    (set(), set(), True, True),
])
def test_bridge_insertion_gate_and_idempotence(closed, reserved, active, success):
    import numpy as np
    from dual_fr3_maniskill.cable.trunking_bridge import TrunkingCableBridge
    names = [f"{side}_fr3_finger_joint{i}" for side in ("left", "right") for i in (1, 2)]
    calls = []
    bridge = NS(failure=None, closed_sides=closed, reserved=reserved,
        config={"gripper_goal_tolerance": .001}, cable_config={"usb": {"finger_position": .0037}},
        get_logger=lambda: NS(info=lambda _: None, error=lambda _: None),
        sim=NS(cable=object() if active else None, indices=dict(zip(names, range(4))),
               positions=np.array([.0037, .0037, 0., 0.]), env=NS(spawn_cable=lambda: calls.append(True))))
    result = TrunkingCableBridge.spawn_cable(bridge, None, NS())
    assert result.success is success
    assert len(calls) == int(success and not active)


def test_native_mtc_insertion_adds_the_attached_object_without_a_robot_motion(tmp_path):
    import copy
    import yaml
    import rclcpp
    from ament_index_python.packages import get_package_share_directory
    from moveit.core.planning_scene import PlanningScene
    from moveit.task_constructor import core
    from dual_fr3_moveit_config.maniskill_resources import build_maniskill_description
    from dual_fr3_trunking_mtc.mtc.task_builder import create_mtc_task
    from dual_fr3_maniskill.cable.model import USB_LINK

    description, semantic = build_maniskill_description(scene="trunking_cable")
    config_dir = Path(get_package_share_directory("dual_fr3_moveit_config")) / "config"
    parameters = {"robot_description": description, "robot_description_semantic": semantic,
                  **yaml.safe_load((config_dir / "kinematics.yaml").read_text())}
    path = tmp_path / "native_cable.yaml"
    path.write_text(yaml.safe_dump({"/**": {"ros__parameters": parameters}}))
    rclcpp.init()
    try:
        node = rclcpp.Node("test_cable_attachment", rclcpp.NodeOptions(
            automatically_declare_parameters_from_overrides=True,
            arguments=["--ros-args", "--params-file", str(path)]))
        owner = core.Task(introspection=False)
        owner.loadRobotModel(node)
        scene = PlanningScene(owner.getRobotModel())
        state = copy.copy(scene.current_state)
        state.set_to_default_values("dual_fr3_arms", "both_ready")
        state.joint_positions = {"left_fr3_finger_joint1": .0037, "right_fr3_finger_joint1": 0.}
        state.update()
        scene.current_state = state
        assert not scene.knows_frame_transform(USB_LINK)
        keypoints = load_keypoints(ROOT / "dual_fr3_trunking_mtc/config/keypoints.yaml")
        plan = build_task_plan(build_segment_plans(keypoints), initial_leader_index=1, initial_follower_index=0)
        spec = cable_specs()[4]
        task, _ = create_mtc_task(node, plan, [spec], start_scene=scene)
        assert task.plan(1)
        solution = task[spec.name].solutions[0]
        assert solution.end.scene.knows_frame_transform(USB_LINK)
        assert not solution.start.scene.knows_frame_transform(USB_LINK)
    finally:
        rclcpp.shutdown()
