"""Exercise measured geometry and intermediate path samples with native MoveIt."""
import copy
import logging
from types import SimpleNamespace as NS

import pytest
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, RobotTrajectory as TrajectoryMessage
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint

from dual_fr3_trunking_mtc.mtc import cached_validation
from dual_fr3_trunking_mtc.mtc.cached_validation import CachedPathValidator


LOGGER = logging.getLogger(__name__)
USB = 'usb_cable_demo_plug'


@pytest.fixture
def model(tmp_path):
    from moveit.core.robot_model import RobotModel
    urdf, srdf = tmp_path / 'robot.urdf', tmp_path / 'robot.srdf'
    urdf.write_text('''<robot name="cache_test">
      <link name="base"/><link name="tcp"/><link name="idle_link"/>
      <link name="wall"><collision><geometry><box size="0.01 0.1 0.01"/>
        </geometry></collision></link>
      <joint name="fixture" type="fixed"><parent link="base"/><child link="wall"/>
        <origin xyz="0.5 0 0.2"/></joint>
      <joint name="travel" type="prismatic"><parent link="base"/><child link="tcp"/>
        <axis xyz="1 0 0"/><limit lower="0" upper="1" effort="1" velocity="1"/></joint>
      <joint name="idle" type="prismatic"><parent link="base"/><child link="idle_link"/>
        <axis xyz="0 1 0"/><limit lower="0" upper="1" effort="1" velocity="1"/></joint>
    </robot>''')
    srdf.write_text('''<robot name="cache_test"><group name="arm"><joint name="travel"/></group>
      <group name="other"><joint name="idle"/></group></robot>''')
    return RobotModel(str(urdf), str(srdf))


def scene_for(model, offset=.3, travel=0., idle=.4):
    from moveit.core.planning_scene import PlanningScene
    scene = PlanningScene(model)
    state = copy.copy(scene.current_state)
    state.set_to_default_values()
    state.joint_positions = {'travel': travel, 'idle': idle}
    state.update()
    scene.current_state = state
    obj = CollisionObject(id=USB, operation=CollisionObject.ADD)
    obj.header.frame_id = 'tcp'
    pose = Pose()
    pose.orientation.w, pose.position.z = 1., offset
    obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.01, .01, .01])]
    obj.primitive_poses = [pose]
    assert scene.process_attached_collision_object(AttachedCollisionObject(link_name='tcp', object=obj))
    return scene


def solution_for(model, positions=(0., 1.)):
    from moveit.core.robot_trajectory import RobotTrajectory
    nominal = scene_for(model, offset=.4, travel=positions[0], idle=0.)
    trajectory = RobotTrajectory(model)
    trajectory.joint_model_group_name = 'arm'
    message = TrajectoryMessage()
    message.joint_trajectory.joint_names = ['travel']
    for index, position in enumerate(positions):
        point = JointTrajectoryPoint(positions=[position], velocities=[.1], accelerations=[0.])
        point.time_from_start.sec = index
        message.joint_trajectory.points.append(point)
    trajectory.set_robot_trajectory_msg(nominal.current_state, message)
    end = copy.copy(nominal)
    state = copy.copy(end.current_state)
    state.joint_positions = {'travel': positions[-1]}
    state.update()
    end.current_state = state
    return NS(trajectory=trajectory, start=NS(scene=nominal), end=NS(scene=end))


def test_measured_attachment_collision_between_original_waypoints_is_rejected(model, caplog):
    solution = solution_for(model)
    measured = scene_for(model, offset=.2)
    # Nominal path and both measured endpoints are clear; only its interior collides.
    assert CachedPathValidator(solution.start.scene, LOGGER).validate('nominal', solution)
    for travel in (0., 1.):
        state = copy.copy(measured.current_state)
        state.joint_positions = {'travel': travel}
        state.update()
        assert measured.is_state_valid(state, '')
    validator = CachedPathValidator(measured, LOGGER)
    assert not validator.validate('transport', solution)
    assert 'waypoint 1' in caplog.text and 'collision' in caplog.text
    assert measured.current_state.joint_positions['travel'] == 0.
    assert solution.end.scene.get_frame_transform(USB)[2, 3] == pytest.approx(.4)


def test_valid_cache_keeps_exact_trajectory_and_uses_measured_inactive_joints(model):
    solution = solution_for(model)
    original = solution.trajectory.get_robot_trajectory_msg()
    measured = scene_for(model)
    validator = CachedPathValidator(measured, LOGGER)
    assert validator.validate('transport', solution)
    assert validator.state.joint_positions['idle'] == pytest.approx(.4)
    assert validator.state.joint_positions['travel'] == 1.
    validator.refresh_attachments()
    assert solution.trajectory.get_robot_trajectory_msg() == original
    for scene in (solution.start.scene, solution.end.scene):
        assert scene.get_frame_transform(USB)[2, 3] == pytest.approx(.3)
    assert measured.current_state.joint_positions['travel'] == 0.


def test_start_drift_is_rejected_without_attempting_to_connect_it(model, caplog):
    validator = CachedPathValidator(scene_for(model, travel=.02), LOGGER)
    assert not validator.validate('descent', solution_for(model))
    assert 'start mismatch' in caplog.text


def test_each_stage_starts_from_previous_validated_end(model):
    validator = CachedPathValidator(scene_for(model), LOGGER)
    assert validator.validate('first', solution_for(model, (0., .4)))
    assert validator.validate('second', solution_for(model, (.4, .9)))
    assert not validator.validate('disconnected', solution_for(model, (.4, .5)))


def test_joint_bounds_are_rejected_even_if_path_is_collision_free(model, caplog):
    validator = CachedPathValidator(scene_for(model), LOGGER)
    assert not validator.validate('transport', solution_for(model, (0., 1.01)))
    assert 'out of bounds' in caplog.text


@pytest.mark.parametrize('continuation_ok', [False, True])
def test_full_suffix_uses_one_measured_scene_and_checks_continuation_before_refresh(model, monkeypatch, continuation_ok):
    measured = scene_for(model)
    solution = solution_for(model)
    calls = []

    def capture(node):
        calls.append(node)
        return NS(scene=measured, model_owner=object())

    def continuation(validator):
        assert validator.state.joint_positions['travel'] == 1.
        assert solution.end.scene.get_frame_transform(USB)[2, 3] == pytest.approx(.4)
        return continuation_ok

    monkeypatch.setattr(cached_validation, 'capture_start_scene', capture)
    cached = [NS(spec=NS(name='transport', mtc_stage_type='MoveTo'), solution=solution)]
    assert cached_validation.validate_cached_after_grasp(
        'node', None, cached, LOGGER, continuation_validator=continuation) is continuation_ok
    assert calls == ['node']
    assert solution.end.scene.get_frame_transform(USB)[2, 3] == pytest.approx(.3 if continuation_ok else .4)


def test_missing_measured_attachment_is_an_error_not_a_collision_free_result(model, monkeypatch):
    from moveit.core.planning_scene import PlanningScene
    monkeypatch.setattr(cached_validation, 'capture_start_scene',
                        lambda node: NS(scene=PlanningScene(model), model_owner=None))
    with pytest.raises(RuntimeError, match='no USB attachment'):
        cached_validation.validate_cached_after_grasp(None, None, [], LOGGER)


def test_native_mtc_execution_message_retains_trajectory_and_measured_attachment(model):
    """MTC includes attached bodies in scene_diff even for a motion-only stage."""
    import rclcpp
    from moveit.task_constructor import core, stages

    rclcpp.init()
    try:
        task = core.Task(introspection=False)
        task.setRobotModel(model)
        start = stages.FixedState('start')
        start.setState(scene_for(model, offset=.4))
        task.add(start)
        move = stages.MoveTo('cached_move', core.JointInterpolationPlanner())
        move.group = 'arm'
        move.setGoal({'travel': 1.})
        task.add(move)
        assert task.plan(1)
        solution = task['cached_move'].solutions[0]
        before = solution.toMsg(None)
        validator = CachedPathValidator(scene_for(model), LOGGER)
        assert validator.validate('cached_move', solution)
        validator.refresh_attachments()
        after = solution.toMsg(None)
        assert len(after.sub_trajectory) == len(before.sub_trajectory) == 1
        assert after.sub_trajectory[0].trajectory == before.sub_trajectory[0].trajectory
        attachment, = after.sub_trajectory[0].scene_diff.robot_state.attached_collision_objects
        assert attachment.object.id == USB
        # MoveIt normalizes primitive_poses into object.pose on serialization.
        assert attachment.object.pose.position.z + attachment.object.primitive_poses[0].position.z == pytest.approx(.3)
    finally:
        rclcpp.shutdown()
