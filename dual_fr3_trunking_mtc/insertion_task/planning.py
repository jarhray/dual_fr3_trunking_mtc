"""Plan the connected right-arm exit and left-arm approach before transport.

The last Cartesian stroke is collision preflight only. PhysX feedback owns the
actual insertion. Keep the originating task alive with every FixedState plan:
the native robot model's kinematics allocators belong to its loader.
"""
import copy
from dataclasses import dataclass
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped, Vector3, Vector3Stamped
from std_msgs.msg import Header
from dual_fr3_maniskill.cable.model import USB_LINK
from dual_fr3_maniskill.usb.insertion import InsertionLimits
from dual_fr3_maniskill.usb.geometry import HOLE, SOCKET_NAME, pose_dict, tcp_goal

from dual_fr3_trunking_mtc.execution.simulation_cable import _observed_pose
from dual_fr3_trunking_mtc.mtc.cached_execution import cache_selected_stages
from dual_fr3_trunking_mtc.mtc.path_length import terminal_path_length_cost
from dual_fr3_trunking_mtc.mtc.planning import PlannedTask
from dual_fr3_trunking_mtc.mtc.task_builder import (
    import_mtc_modules,
    create_motion_planners,
    _identity_ik_frame,
)


@dataclass
class InsertionApproach:
    planned: PlannedTask
    motions: dict

    def execute(self, name):
        # A failed execution can have moved the robot. Never replay this cache.
        result = self.planned.task.execute(self.motions[name])
        if not result:
            raise RuntimeError(name + ': motion execution failed ' + str(getattr(result, 'val', result)))


def approach_goals(scene, config):
    """TCP goals in the model frame, using the USB grasp in this planning scene."""
    frames = (SOCKET_NAME, 'left_fr3_hand_tcp', USB_LINK)
    for frame in frames:
        if not scene.knows_frame_transform(frame):
            raise ValueError('Missing insertion planning frame: ' + frame)
    # Each collision object has one mesh and an unset object.pose. MoveIt
    # promotes mesh_poses[0] to the object frame, including the measured mount.
    base, tcp, usb = (
        scene.get_frame_transform(frame)
        for frame in frames
    )
    goals = []
    limits = InsertionLimits.read(config)
    for depth in (-limits.preinsert_m, limits.target_depth_m):
        transform = tcp_goal(base, tcp, usb, depth, config.get('hole_center_m', HOLE))
        goals.append(PoseStamped(
            header=Header(frame_id=scene.planning_frame),
            pose=_observed_pose({'pose': pose_dict(transform)}, 'pose')))
    return goals


def build_approach_task(node, scene, config):
    _, core, stages = import_mtc_modules()
    task = core.Task()
    task.name = 'insertion_approach_and_collision_preflight'
    task.setRobotModel(scene.robot_model)
    start = stages.FixedState('transport_end_state')
    start.setState(copy.copy(scene))
    task.add(start)
    cart, joint, ompl = create_motion_planners(core, node, .001, .08, .08, 2.)
    retreat, _, _ = create_motion_planners(core, node, .001, .05, .05, 2.)
    ratio = float(config.get('max_path_length_ratio', 1.5))
    timeout = float(config.get('planning_timeout_s', 15.))
    specs = []

    def add(stage):
        specs.append(SimpleNamespace(name=stage.name, executable=True, mtc_stage_type='MoveTo'))
        task.add(stage)  # ownership transfers to C++; do not read stage afterwards

    def move(name, group, goal, planner, link=None):
        stage = stages.MoveTo(name, planner)
        stage.group = group
        if link:
            stage.ik_frame = _identity_ik_frame(link)
        stage.setGoal(goal)
        stage.timeout = timeout
        stage.setCostTerm(terminal_path_length_cost(
            link or group.removesuffix('_arm') + '_hand_tcp', goal, ratio, name, group=group))
        add(stage)

    # Preview the same opening sent through the gripper action at execution.
    opening = stages.MoveTo('preview_right_release', joint)
    opening.group = 'right_fr3_hand'
    opening.setGoal({'right_fr3_finger_joint1':
                     float(config.get('release_width_m', {}).get('right', .03)) / 2})
    task.add(opening)
    withdrawal = stages.MoveRelative('withdraw_right', retreat)
    withdrawal.group = 'right_fr3_arm'
    withdrawal.ik_frame = _identity_ik_frame('right_fr3_hand_tcp')
    direction = Vector3Stamped(header=Header(frame_id='right_fr3_hand_tcp'),
                              vector=Vector3(z=-float(config.get('retreat_m', .05))))
    withdrawal.setDirection(direction)
    withdrawal.setCostTerm(terminal_path_length_cost(
        'right_fr3_hand_tcp', direction, ratio, 'right_socket_withdraw'))
    add(withdrawal)
    move('right_return_ready', 'right_fr3_arm', 'ready', ompl)
    goal, end = approach_goals(scene, config)
    lift = float(config.get('approach_lift_m', 0.))
    if lift:
        high = copy.deepcopy(goal)
        high.pose.position.z += lift
        move('socket_above_approach', 'left_fr3_arm', high, ompl, 'left_fr3_hand_tcp')
    move('socket_preinsert', 'left_fr3_arm', goal, cart if lift else ompl, 'left_fr3_hand_tcp')
    expected = stages.ModifyPlanningScene('expected_USB_socket_contact_only')
    expected.allowCollisions(USB_LINK, [SOCKET_NAME], True)
    task.add(expected)
    move('insertion_collision_preflight', 'left_fr3_arm', end, cart, 'left_fr3_hand_tcp')
    return task, tuple(specs)


def plan_approach(node, scene, config, logger, *, model_owner):
    attempts = int(config.get('planning_attempts', 10))
    for attempt in range(1, attempts + 1):
        task, specs = build_approach_task(node, scene, config)
        logger.info('connected insertion approach planning attempt %d/%d', attempt, attempts)
        if task.plan(1) and task.solutions:
            planned = PlannedTask(task, task.solutions[0], specs, model_owner=model_owner)
            motions = {item.spec.name: item.solution for item in cache_selected_stages(planned)}
            # Collision-only stroke must never enter the executable cache.
            del motions['insertion_collision_preflight']
            return InsertionApproach(planned, motions)
    logger.error('connected insertion approach planning exhausted; no terminal motion issued')
    return None
