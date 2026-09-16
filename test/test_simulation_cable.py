from pathlib import Path
import json
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


def cable_specs(direction="reverse"):
    keypoints = load_keypoints(ROOT / "dual_fr3_trunking_mtc/config/keypoints.yaml")
    plan = build_task_plan(build_segment_plans(keypoints), initial_leader_index=1, initial_follower_index=0)
    return build_preparation_stage_specs(plan, PreparationConfig(
        simulation_cable_config=CONFIG, leader_orientation_direction=direction))


def test_spawn_precedes_contact_closure_and_verification_precedes_transport():
    specs = cable_specs()
    assert [s.mtc_stage_type for s in specs] == [
        "GripperOperation", "GripperOperation", "SimulationCable", "MoveTo", "MoveTo",
        "GripperOperation", "GripperOperation", "SimulationCable", "Merger"]
    assert specs[0].gripper_width_override > .0076
    assert specs[2].cable_operation == "spawn"
    assert specs[5].gripper_width_override == specs[6].gripper_width_override == 0.
    assert specs[5].gripper_action == "grasp"
    assert specs[7].cable_operation == "release_verify"
    assert [s.stage_index for s in specs] == list(range(9))
    assert all(s.stage_index == 8 for s in specs[-1].children)


def test_usb_only_preserves_both_arm_preparation_and_original_motion_stages():
    from dual_fr3_trunking_mtc.stages.compiler import build_mtc_stage_specs
    keypoints = load_keypoints(ROOT / "dual_fr3_trunking_mtc/config/keypoints.yaml")
    plan = build_task_plan(build_segment_plans(keypoints), initial_leader_index=1, initial_follower_index=0)
    normal = build_mtc_stage_specs(plan, preparation_config=PreparationConfig(
        simulation_cable_config=CONFIG, load_cable=True))
    usb_only = build_mtc_stage_specs(plan, preparation_config=PreparationConfig(
        simulation_cable_config=CONFIG, load_cable=False))
    assert usb_only == normal
    assert any(s.actor == "follower" for s in usb_only)
    assert usb_only[7].cable_operation == "release_verify"
    assert usb_only[8].mtc_stage_type == "Merger"
    assert any(s.phase == "formal" for s in usb_only[9:])


@pytest.mark.parametrize("direction,sign", [("forward", 1.), ("reverse", -1.)])
def test_planning_scene_attaches_usb_at_the_grip_in_the_leader_direction(direction, sign):
    from dual_fr3_maniskill.cable.planning_scene import attached_usb_scene
    from dual_fr3_maniskill.cable.model import USB_LINK
    from scipy.spatial.transform import Rotation
    import numpy as np
    spec = cable_specs(direction)[2]
    assert spec.cable_orientation_direction == direction
    scene = attached_usb_scene(CONFIG, orientation_direction=spec.cable_orientation_direction)
    attachment, = scene.robot_state.attached_collision_objects
    assert attachment.link_name == attachment.object.header.frame_id == "left_fr3_hand_tcp"
    assert attachment.object.id == USB_LINK
    assert "right_fr3_hand_tcp" not in attachment.touch_links
    q = attachment.object.mesh_poses[0].orientation
    rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    np.testing.assert_allclose(rotation @ [0., 1., 0.], [sign, 0., 0.], atol=1e-10)
    p = attachment.object.mesh_poses[0].position
    np.testing.assert_allclose([p.x, p.y, p.z], [0., 0., .012])
    assert scene.is_diff and scene.robot_state.is_diff


def observed_status():
    return dict(state="stable", external_support=False, relative_frame="left_fr3_hand_tcp",
                world_pose=dict(frame="world", position_m=[.4, .5, .6], quaternion_wxyz=[1., 0., 0., 0.]),
                relative_pose=dict(position_m=[.001, 0., .0121], quaternion_wxyz=[1., 0., 0., 0.]))


@pytest.mark.parametrize("operation,expected", [("spawn", ["prepare", "spawn", "status", "apply"]),
                                                   ("release_verify", ["release", "verify", "status", "apply"])])
def test_physical_verification_precedes_planning_attachment(operation, expected):
    from dual_fr3_trunking_mtc.simulation_cable import SimulationCableController
    calls = []
    apply = NS(wait_for_service=lambda **_: True)
    def call(client, request, **kwargs):
        calls.append("apply" if client is apply else client)
        if client == "status":
            assert kwargs.get("require_success", True) is (operation != "spawn")
            return NS(message=json.dumps(observed_status()))
        if client is apply:
            attached = request.scene.robot_state.attached_collision_objects
            assert len(attached) == 1
            from moveit_msgs.msg import CollisionObject
            assert attached[0].object.operation == (
                CollisionObject.ADD if operation == "release_verify" else CollisionObject.REMOVE)
            assert bool(request.scene.world.collision_objects) is (operation == "spawn")
            if operation == "spawn":
                obj = request.scene.world.collision_objects[0]
                assert obj.header.frame_id == "world"
                assert obj.mesh_poses[0].position.x == .4
            else:
                assert attached[0].object.mesh_poses[0].position.x == .001
                assert attached[0].object.mesh_poses[0].position.z == .0121
        return NS(message="ok")
    controller = NS(apply=apply, prepare="prepare", spawn="spawn", release="release", verify="verify", status="status", _call=call,
                    node=NS(get_logger=lambda: NS(info=lambda _: None)))
    spec = next(s for s in cable_specs() if s.mtc_stage_type == "SimulationCable" and s.cable_operation == operation)
    assert SimulationCableController.execute(controller, spec)
    assert calls == expected


def test_repeated_spawn_uses_stationary_usb_world_observation_after_tcp_moves():
    from dual_fr3_trunking_mtc.simulation_cable import SimulationCableController
    apply = NS(wait_for_service=lambda **_: True)
    scenes = []
    snapshot = observed_status()
    snapshot.update(state="supported", external_support=True)
    def call(client, request, **kwargs):
        if client == "status":
            assert kwargs["require_success"] is False
            return NS(success=False, message=json.dumps(snapshot))
        if client is apply:
            scenes.append(request.scene)
        return NS(success=True, message="USB already created; unchanged")
    controller = NS(apply=apply, prepare="prepare", spawn="spawn", status="status", _call=call,
                    node=NS(get_logger=lambda: NS(info=lambda _: None)))
    spec = next(s for s in cable_specs() if s.mtc_stage_type == "SimulationCable" and s.cable_operation == "spawn")
    SimulationCableController.execute(controller, spec)
    # Relative USB location changes when TCP moves; the actual world pose does not.
    snapshot["relative_pose"]["position_m"] = [-.2, .1, .012]
    SimulationCableController.execute(controller, spec)
    first, second = [scene.world.collision_objects[0] for scene in scenes]
    assert first.header.frame_id == second.header.frame_id == "world"
    assert first.mesh_poses == second.mesh_poses
    assert second.mesh_poses[0].position.x == .4


@pytest.mark.parametrize("world_pose", [None, {"frame": "tcp"},
    dict(frame="world", position_m=[float("nan"), 0., 0.], quaternion_wxyz=[1., 0., 0., 0.]),
    dict(frame="world", position_m=[0., 0., 0.], quaternion_wxyz=[0., 0., 0., 0.])])
def test_invalid_spawn_observation_does_not_change_planning_scene(world_pose):
    from dual_fr3_trunking_mtc.simulation_cable import SimulationCableController
    apply = NS(wait_for_service=lambda **_: True)
    calls = []
    def call(client, request, **kwargs):
        calls.append(client)
        return NS(message=json.dumps(dict(world_pose=world_pose)) if client == "status" else "created")
    controller = NS(apply=apply, prepare="prepare", spawn="spawn", status="status", _call=call,
                    node=NS(get_logger=lambda: NS(info=lambda _: None)))
    spec = next(s for s in cable_specs() if s.mtc_stage_type == "SimulationCable" and s.cable_operation == "spawn")
    with pytest.raises(RuntimeError):
        SimulationCableController.execute(controller, spec)
    assert apply not in calls


@pytest.mark.parametrize("invalid", ["state", "support", "frame", "pose"])
def test_verification_followed_by_invalid_status_never_attaches(invalid):
    from dual_fr3_trunking_mtc.simulation_cable import SimulationCableController
    snapshot = observed_status()
    if invalid == "state":
        snapshot["state"] = "slipping"
    elif invalid == "support":
        snapshot["external_support"] = True
    elif invalid == "frame":
        snapshot["relative_frame"] = "right_fr3_hand_tcp"
    else:
        snapshot["relative_pose"] = None
    apply = NS(wait_for_service=lambda **_: True)
    calls = []
    def call(client, request, **kwargs):
        calls.append(client)
        return NS(message=json.dumps(snapshot) if client == "status" else "ok")
    controller = NS(apply=apply, release="release", verify="verify", status="status", _call=call,
                    node=NS(get_logger=lambda: NS(info=lambda _: None)))
    spec = next(s for s in cable_specs() if s.cable_operation == "release_verify")
    with pytest.raises(RuntimeError):
        SimulationCableController.execute(controller, spec)
    assert calls == ["release", "verify", "status"]


@pytest.mark.parametrize("failed", ["release", "verify"])
def test_contact_or_verification_failure_never_attaches_in_planning(failed):
    from dual_fr3_trunking_mtc.simulation_cable import SimulationCableController
    calls = []
    apply = NS(wait_for_service=lambda **_: True)
    def call(client, request):
        calls.append(client)
        if client == failed:
            raise RuntimeError("insufficient contact or unstable grasp")
        return NS(message="ok")
    controller = NS(apply=apply, release="release", verify="verify", _call=call,
                    node=NS(get_logger=lambda: NS(info=lambda _: None)))
    spec = next(s for s in cable_specs() if s.cable_operation == "release_verify")
    with pytest.raises(RuntimeError, match="unstable grasp"):
        SimulationCableController.execute(controller, spec)
    assert apply not in calls


@pytest.mark.parametrize("direction,sign", [("forward", 1.), ("reverse", -1.)])
def test_native_mtc_insertion_adds_the_attached_object_without_a_robot_motion(tmp_path, direction, sign):
    import copy
    import numpy as np
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
        spec = cable_specs(direction)[7]
        task, _ = create_mtc_task(node, plan, [spec], start_scene=scene)
        assert task.plan(1)
        solution = task[spec.name].solutions[0]
        assert solution.end.scene.knows_frame_transform(USB_LINK)
        assert not solution.start.scene.knows_frame_transform(USB_LINK)
        end = solution.end.scene
        mount = np.linalg.inv(end.get_frame_transform("left_fr3_hand_tcp")) @ end.get_frame_transform(USB_LINK)
        np.testing.assert_allclose(mount[:3, 1], [sign, 0., 0.], atol=1e-10)
        np.testing.assert_allclose(mount[:3, 3], [0., 0., .012], atol=1e-10)
    finally:
        rclcpp.shutdown()


def test_preposition_targets_match_keypoints_height_and_approach_tool_pose():
    from dual_fr3_trunking_mtc.mtc.task_builder import preparation_pose_goal
    points = load_keypoints(ROOT / "dual_fr3_trunking_mtc/config/keypoints.yaml")
    plan = build_task_plan(build_segment_plans(points), initial_leader_index=1, initial_follower_index=0)
    config = PreparationConfig(simulation_cable_config=CONFIG, approach_height=.09,
                               tool_roll=2.9, tool_pitch=.12)
    specs = build_preparation_stage_specs(plan, config)
    spawn = next(s for s in specs if s.mtc_stage_type == "SimulationCable" and s.cable_operation == "spawn")
    for side, actor in (("left", "leader"), ("right", "follower")):
        approach = next(s for s in specs if s.actor == actor and s.primitive == "move_above_initial_keypoint")
        assert spawn.stage_index < approach.stage_index
        pose = preparation_pose_goal(plan, approach, config.tool_roll, config.tool_pitch)
        target = spawn.cable_preparation_poses[side]
        assert target["frame"] == pose.header.frame_id
        assert target["position_m"] == pytest.approx([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
        q = pose.pose.orientation
        assert target["quaternion_wxyz"] == pytest.approx([q.w, q.x, q.y, q.z])
    assert specs[-1].primitive == "dual_cartesian_descent"
    from dual_fr3_trunking_mtc.stages.compiler import build_mtc_stage_specs
    complete = build_mtc_stage_specs(plan, preparation_config=config)
    assert complete[len(specs)].phase == "formal"


def test_preposition_planning_object_uses_target_frame_instead_of_live_tcp():
    from dual_fr3_maniskill.cable.planning_scene import usb_collision_object
    spec = cable_specs()[2]
    target = spec.cable_preparation_poses["left"]
    obj = usb_collision_object(CONFIG, preparation_tcp_pose=target)
    assert obj.header.frame_id == target["frame"]
    p = obj.mesh_poses[0].position
    # roll=pi: local +Z is downward; default USB grip is 12 mm below TCP.
    assert [p.x, p.y, p.z] == pytest.approx([*target["position_m"][:2], target["position_m"][2]-.012])
