"""Real insertion adapters under deterministic replay and mocked ROS endpoints.

These tests neither initialize ROS nor address any robot controller.
"""
import copy
import json
import sys
import types
from types import SimpleNamespace as NS

import numpy as np
import pytest
import yaml
from transforms3d.quaternions import mat2quat
from transforms3d.axangles import axangle2mat

from dual_fr3_maniskill.usb.geometry import HOLE, TIP, USB_IN_SOCKET
from dual_fr3_trunking_mtc.insertion_task.local_kinematics import LocalKinematics, pose_error
from dual_fr3_trunking_mtc.insertion_task.real_session import RealInsertionSession, load_real_config
from dual_fr3_trunking_mtc.insertion_task import real_runtime


def configuration():
    return dict(insertion=dict(
        observation_mode='calibrated_estimate', retain_after_success=False,
        release_after_retention=False, return_after_release=False,
        calibration=dict(calibration_id='offline_test_calibration', verified=True,
            world_T_socket=np.eye(4).tolist(), tcp_T_usb=np.eye(4).tolist(),
            max_age_s=.05, max_gripper_age_s=.2,
            closed_width_range_m=[.015, .025], max_translation_uncertainty_m=.0001,
            max_angle_uncertainty_rad=.01, uncertainty_translation_m=.00005,
            uncertainty_angle_rad=.005, max_sample_gap_s=.05,
            wrench_sign=1., wrench_bias=[0.] * 6, grasp_assumption_valid=True)),
        real_runtime=dict(frequency_hz=100., max_dt_s=.04, command_valid_for_s=.04,
            controller_state_max_age_s=.05, fk_translation_tolerance_m=.0005,
            fk_angle_tolerance_rad=.005, heartbeat_timeout_s=1.,
            local_collision_check='per_step', limits_verified=True,
            world_T_base=np.eye(4).tolist(), ee_T_tcp=np.eye(4).tolist(),
            collision_timeout_s=.02, right_ready_q=[0.] * 7,
            right_min_half_width_m=.03))


def source_sample(now, *, tcp=None, axial_force=0., **changes):
    if tcp is None:
        tcp = np.eye(4)
        tcp[:3, :3] = USB_IN_SOCKET
        tcp[:3, 3] = HOLE + [.008, 0., 0.] - USB_IN_SOCKET @ TIP
    k = np.eye(4)
    k[:3, 3] = HOLE
    result = dict(time_s=now, world_T_tcp=np.array(tcp, copy=True),
        tcp_linear_velocity_world=[0.] * 3, tcp_angular_velocity_world=[0.] * 3,
        wrench_world_at_k=[axial_force, 0., 0., 0., 0., 0.], world_T_k=k,
        wrench_time_s=now, gripper_time_s=now, gripper_width_m=.02,
        gripper_healthy=True, gripper_grasped=None,
        robot_healthy=True, wrench_available=True)
    result.update(changes)
    return result


def write_config(tmp_path, config):
    path = tmp_path / 'real.yaml'
    path.write_text(yaml.safe_dump(config), encoding='utf-8')
    return path


def test_real_configuration_supports_unverified_read_only_inspection(tmp_path):
    config = configuration()
    config['insertion']['calibration']['verified'] = False
    loaded = load_real_config(write_config(tmp_path, config))
    session = RealInsertionSession(loaded)
    assert session.observe(1., source_sample(1.))['observation_valid']
    with pytest.raises(ValueError, match='unverified'):
        session.begin(1., source_sample(1.), align=False)


@pytest.mark.parametrize('section,key,value', [
    ('insertion', 'observation_mode', 'simulation_truth'),
    ('insertion', 'retain_after_success', True),
    ('insertion', 'release_after_retention', True),
    ('insertion', 'return_after_release', True),
    ('real_runtime', 'frequency_hz', True),
    ('real_runtime', 'command_valid_for_s', float('nan')),
    ('real_runtime', 'max_dt_s', .001),
    ('real_runtime', 'local_collision_check', 'entry'),
    ('real_runtime', 'collision_timeout_s', -1.),
    ('real_runtime', 'world_T_base', np.diag([2., 1., 1., 1.]).tolist()),
    ('real_runtime', 'ee_T_tcp', np.eye(3).tolist()),
    ('real_runtime', 'right_ready_q', [0.] * 6),
])
def test_real_configuration_rejects_execution_ambiguities(tmp_path, section, key, value):
    config = configuration()
    config[section][key] = value
    with pytest.raises(ValueError):
        load_real_config(write_config(tmp_path, config))


def test_real_session_requires_verified_runtime_limits():
    config = configuration()
    config['real_runtime']['limits_verified'] = False
    session = RealInsertionSession(config)
    with pytest.raises(RuntimeError, match='limits'):
        session.begin(1., source_sample(1.), align=False)
    assert session.goal is None


def test_alignment_goal_is_bounded_and_committed_only_after_external_approval():
    session = RealInsertionSession(configuration())
    sample = source_sample(1.)
    sample['world_T_tcp'][1, 3] += .0005
    session.begin(1., sample, align=True)
    original = session.goal.copy()
    next_sample = source_sample(1.01, tcp=original)
    goal = session.next_goal(1.01, next_sample)
    assert goal is not None
    assert 0. < original[1, 3] - goal[1, 3] <= .00001000001
    np.testing.assert_array_equal(session.goal, original)
    session.accept_goal(goal)
    np.testing.assert_array_equal(session.goal, goal)


def test_force_below_overload_can_stall_position_policy_without_false_success():
    session = RealInsertionSession(configuration())
    session.begin(1., source_sample(1.), align=False)
    for index in range(1, 180):
        now = 1. + index * .01
        goal = session.next_goal(now, source_sample(now, tcp=session.goal, axial_force=3.))
        if goal is None:
            break
        session.accept_goal(goal)
    assert session.policy.state == 'blocked'
    assert session.policy.reason == 'no_measured_tip_progress'
    assert not session.policy.insertion_success
    assert session.policy.speed == 0.


def test_replayed_insertion_reports_only_estimated_success_and_keeps_grip():
    session = RealInsertionSession(configuration())
    session.begin(1., source_sample(1.), align=False)
    for index in range(1, 2501):
        now = 1. + index * .01
        goal = session.next_goal(now, source_sample(now, tcp=session.goal))
        if goal is None:
            break
        session.accept_goal(goal)
    snapshot = session.snapshot()
    assert snapshot['state'] == 'inserted_unretained'
    assert snapshot['outcome'] == 'estimated_reached'
    assert snapshot['physical_success_verified'] is False
    assert snapshot['retention_active'] is False
    assert snapshot['grippers_released'] is False
    assert snapshot['observation']['grasp_valid'] is None


@pytest.mark.parametrize('dt', [0., -.01, .1])
def test_real_session_stops_on_invalid_control_period(dt):
    session = RealInsertionSession(configuration())
    session.begin(1., source_sample(1.), align=False)
    assert session.next_goal(1. + dt, source_sample(1. + dt)) is None
    assert session.policy.state == 'blocked'
    assert session.policy.reason == 'policy_clock_or_update_gap'


def test_real_session_stops_with_stale_observation_even_with_fresh_loop_clock():
    session = RealInsertionSession(configuration())
    session.begin(1., source_sample(1.), align=False)
    assert session.next_goal(1.01, source_sample(.8)) is None
    assert session.policy.state == 'feedback_unavailable'


def test_real_session_stops_before_integrating_large_tracking_error():
    session = RealInsertionSession(configuration())
    session.begin(1., source_sample(1.), align=False)
    sample = source_sample(1.01)
    sample['world_T_tcp'][0, 3] += .002
    assert session.next_goal(1.01, sample) is None
    assert session.policy.reason == 'joint_drive_tracking_error'


def seven_joint_urdf():
    parts = ['<robot name="offline">']
    for index in range(7):
        kind = 'prismatic' if index < 3 else 'revolute'
        axis = np.eye(3, dtype=int)[index % 3]
        xyz = '.03 -.02 .05' if index else '.2 .1 .3'
        parent = 'world' if index == 0 else f'link{index}'
        parts.append(f'<joint name="joint{index + 1}" type="{kind}">'
            f'<parent link="{parent}"/><child link="link{index + 1}"/>'
            f'<origin xyz="{xyz}" rpy=".1 -.05 .07"/>'
            f'<axis xyz="{" ".join(map(str, axis))}"/>'
            '<limit lower="-2" upper="2"/></joint>')
    parts.append('<joint name="tool" type="fixed"><parent link="link7"/>'
                 '<child link="tcp"/><origin xyz=".02 .01 .1"/></joint></robot>')
    return ''.join(parts)


def test_local_jacobian_matches_finite_difference_of_real_chain_transforms():
    kinematics = LocalKinematics(seven_joint_urdf(), tip='tcp')
    q = np.array([.01, -.02, .03, .2, -.3, .4, .1])
    transform, jacobian = kinematics.forward(q, jacobian=True)
    step = 1.e-6
    measured = np.empty((6, 7))
    for i in range(7):
        delta = np.zeros(7)
        delta[i] = step
        before, after = kinematics.forward(q - delta), kinematics.forward(q + delta)
        rotation_derivative = (after[:3, :3] - before[:3, :3]) / (2. * step)
        omega = rotation_derivative @ transform[:3, :3].T
        measured[:, i] = np.r_[(after[:3, 3] - before[:3, 3]) / (2. * step),
                               omega[2, 1], omega[0, 2], omega[1, 0]]
    np.testing.assert_allclose(jacobian, measured, atol=2.e-8, rtol=2.e-7)
    np.testing.assert_allclose(transform[3], [0., 0., 0., 1.])


@pytest.mark.parametrize('angle', [2.e-6, 1.e-9, .2, np.pi - 1.e-7])
def test_rotation_error_is_stable_for_small_and_near_half_turn_angles(angle):
    axis = np.array([.94216938, .23930098, .23463143])
    axis /= np.linalg.norm(axis)
    goal = np.eye(4)
    goal[:3, :3] = axangle2mat(axis, angle)
    np.testing.assert_allclose(pose_error(np.eye(4), goal)[3:], axis * angle,
                               atol=2.e-9, rtol=2.e-8)


def test_local_ik_reaches_small_goal_within_seed_envelope():
    kinematics = LocalKinematics(seven_joint_urdf(), tip='tcp')
    seed = np.array([.01, -.02, .03, .2, -.3, .4, .1])
    target_q = seed + np.array([.0002, -.0001, .0001, .0002, -.0002, .0001, 0.])
    goal = kinematics.forward(target_q)
    solution = kinematics.solve(goal, seed, .001)
    assert np.max(abs(solution - seed)) <= .001
    error = pose_error(kinematics.forward(solution), goal)
    assert np.linalg.norm(error[:3]) <= 1.e-5
    assert np.linalg.norm(error[3:]) <= 1.e-4


def test_unreachable_ik_does_not_return_a_clipped_non_solution():
    kinematics = LocalKinematics(seven_joint_urdf(), tip='tcp')
    seed = np.zeros(7)
    goal = kinematics.forward(seed)
    goal[0, 3] += .1
    with pytest.raises(RuntimeError, match='envelope'):
        kinematics.solve(goal, seed, .0001)
    with pytest.raises(ValueError, match='seed'):
        kinematics.solve(goal, np.ones(7) * 3., .1)


@pytest.mark.parametrize('kind', ['scale', 'reflection', 'last_row'])
def test_local_ik_rejects_non_rigid_targets(kind):
    kinematics = LocalKinematics(seven_joint_urdf(), tip='tcp')
    goal = kinematics.forward(np.zeros(7))
    if kind == 'scale':
        goal[:3, 0] *= 1.01
    elif kind == 'reflection':
        goal[:3, 0] *= -1.
    else:
        goal[3, 0] = .1
    with pytest.raises(ValueError, match='target'):
        kinematics.solve(goal, np.zeros(7), .001)


@pytest.mark.parametrize('change', ['names', 'mimic', 'continuous'])
def test_local_chain_rejects_unexpected_control_topology(change):
    description = seven_joint_urdf()
    names = None
    if change == 'names':
        names = [f'joint{i}' for i in range(7, 0, -1)]
    elif change == 'mimic':
        description = description.replace('<limit', '<mimic joint="joint2"/><limit', 1)
    else:
        description = description.replace('type="prismatic"', 'type="continuous"', 1)
    with pytest.raises(ValueError):
        LocalKinematics(description, tip='tcp', joint_names=names)


@pytest.fixture
def fake_ros(monkeypatch):
    """Minimal endpoint doubles; make_node remains the implementation under test."""
    clock = dict(mono=1., ros=100.)
    monkeypatch.setattr(real_runtime.time, 'monotonic', lambda: clock['mono'])

    def advance_clock(duration):
        clock['mono'] += duration
        clock['ros'] += duration

    monkeypatch.setattr(real_runtime.time, 'sleep', advance_clock)

    class Message(NS):
        pass

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    class Client:
        def __init__(self, name):
            self.srv_name, self.requests = name, []
            self.complete = True
            self.next_result = NS(success=True, ok=False)

        def wait_for_service(self, **_kwargs):
            return True

        def call_async(self, request):
            self.requests.append(request)
            return NS(done=lambda: self.complete, result=lambda: self.next_result,
                add_done_callback=lambda callback: callback(None) if self.complete else None)

    class Node:
        def __init__(self, name):
            self.subscriptions = {}
            self.warnings = []

        def get_parameter(self, name):
            return NS(value=False)

        def create_publisher(self, *_args):
            return Publisher()

        def create_client(self, _kind, name, **_kwargs):
            return Client(name)

        def create_subscription(self, _kind, name, callback, *_args, **_kwargs):
            self.subscriptions[name] = callback

        def create_service(self, *_args, **_kwargs):
            return NS()

        def create_timer(self, *_args, **_kwargs):
            return NS()

        def get_clock(self):
            stamp = NS(sec=int(clock['ros']), nanosec=int(clock['ros'] % 1. * 1.e9))
            return NS(now=lambda: NS(nanoseconds=int(clock['ros'] * 1.e9), to_msg=lambda: stamp))

        def get_logger(self):
            return NS(warning=lambda text, **_: self.warnings.append(text))

    class Request(NS):
        STRICT = 2

        def __init__(self, **kwargs):
            super().__init__(timeout=NS(sec=0), **kwargs)

    class SceneComponents(Message):
        WORLD_OBJECT_GEOMETRY = 16
        ROBOT_STATE_ATTACHED_OBJECTS = 4

    service = NS(Request=Request)
    modules = {
        'rclpy': {}, 'rclpy.node': {'Node': Node},
        'rclpy.callback_groups': {'ReentrantCallbackGroup': Message},
        'rclpy.qos': {'qos_profile_sensor_data': None},
        'std_msgs': {}, 'std_msgs.msg': {'String': Message},
        'std_srvs': {}, 'std_srvs.srv': {'Trigger': service},
        'sensor_msgs': {}, 'sensor_msgs.msg': {'JointState': Message},
        'franka_msgs': {}, 'franka_msgs.msg': {'FrankaRobotState': Message},
        'controller_manager_msgs': {}, 'controller_manager_msgs.srv': {
            'SwitchController': service, 'ListControllers': service},
        'rcl_interfaces': {}, 'rcl_interfaces.srv': {'GetParameters': service},
        'moveit_msgs': {}, 'moveit_msgs.srv': {
            'GetStateValidity': service, 'GetPlanningScene': service},
        'moveit_msgs.msg': {'PlanningSceneComponents': SceneComponents},
        'dual_fr3_moveit_config': {},
        'dual_fr3_moveit_config.msg': {'JointTarget': Message, 'JointTargetState': Message},
    }
    for name, attributes in modules.items():
        module = types.ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    return clock


def test_read_only_align_cannot_call_a_controller_service(fake_ros):
    node = real_runtime.make_node(configuration(), allow_execution=False)
    node.sample = lambda: (np.zeros(7), source_sample(1.))
    node.right_check = lambda ready: None
    response = node.command('align', NS())
    assert not response.success and 'read-only' in response.message
    assert node.switch_client.requests == []
    assert node.targets.messages == []
    assert node.owner is False


def test_aligned_hold_stops_on_stale_feedback_instead_of_renewing_targets(fake_ros):
    node = real_runtime.make_node(configuration(), allow_execution=True)
    node.session.policy.state = 'aligned'
    node.owner = node.running = True
    node.controller_session = 17
    node.controller = (executor_state(), fake_ros['mono'])
    node.q_reference = np.zeros(7)
    node.sample = lambda: (np.zeros(7), source_sample(.5))
    commands = []
    node.publish_target = lambda q: commands.append(q)
    node.tick()
    assert commands == []
    assert not node.running
    assert len(node.stop_client.requests) == 1
    assert node.session.policy.reason == 'invalid_or_missing_sensor'


def test_executor_state_expiry_prevents_command_publication(fake_ros):
    node = real_runtime.make_node(configuration(), allow_execution=True)
    node.controller_session = 'session-one'
    node.controller = (NS(state='TRACKING', session='session-one', reason=''), .9)
    with pytest.raises(RuntimeError, match='stale'):
        node.publish_target(np.zeros(7))
    assert node.targets.messages == []


@pytest.mark.parametrize('source_time', [0., 99., 101.])
def test_stale_or_future_executor_source_stamp_cannot_be_freshened_by_receipt(fake_ros, source_time):
    node = real_runtime.make_node(configuration(), allow_execution=True)
    node.controller_session = 17
    state = NS(state='TRACKING', session=17, reason='',
               stamp=NS(sec=int(source_time), nanosec=0))
    node.controller = (state, fake_ros['mono'])
    with pytest.raises(RuntimeError):
        node.publish_target(np.zeros(7))
    assert node.targets.messages == []


def test_executor_session_change_prevents_command_publication(fake_ros):
    node = real_runtime.make_node(configuration(), allow_execution=True)
    node.controller_session = 17
    node.controller = (NS(state='HOLDING', session=18, reason='reactivated',
        stamp=NS(sec=100, nanosec=0)), fake_ros['mono'])
    with pytest.raises(RuntimeError, match='ownership'):
        node.publish_target(np.zeros(7))
    assert node.targets.messages == []


@pytest.mark.parametrize('failure', ['denied', 'timeout'])
def test_failed_or_uncertain_switch_requests_stop_and_reconciles_ownership(fake_ros, failure):
    node = real_runtime.make_node(configuration(), allow_execution=True)
    node.sample = lambda: (np.zeros(7), source_sample(1.))
    node.right_check = lambda ready: None
    node.validate_joint_goal = lambda start, goal: None
    node.validate_scene = lambda: None
    switches = []

    def call(client, request, **_kwargs):
        if client is node.parameters_client:
            if request.names[0] == 'command_timeout_s':
                return NS(values=[NS(type=3, double_value=.1),
                    NS(type=8, double_array_value=[.2] * 7), NS(type=3, double_value=.003)])
            return NS(values=[NS(type=1, bool_value=True) for _ in range(3)])
        if client is node.list_client:
            return NS(controller=[NS(name='left_fr3_arm_controller', state='active'),
                NS(name='left_insertion_controller', state='inactive')])
        if client is node.switch_client:
            switches.append(request)
            if failure == 'timeout':
                raise RuntimeError('injected switch timeout')
            return NS(ok=False)
        raise AssertionError('Unexpected service call')

    node.call = call
    response = node.command('align', NS())
    assert not response.success
    assert len(switches) == 1  # An uncertain switch must never be retried.
    assert len(node.stop_client.requests) == 1
    assert node.targets.messages == []
    assert not node.owner and not node.running
    assert node.controller_inventory['left_fr3_arm_controller'] == 'active'


def test_cancel_requests_hold_without_releasing_control_ownership(fake_ros):
    node = real_runtime.make_node(configuration(), allow_execution=True)
    node.owner = node.running = True
    node.session.policy.state = 'feedback_advance'
    response = node.command('cancel', NS())
    assert response.success
    assert node.owner and not node.running
    assert node.session.policy.reason == 'client_cancel'
    assert len(node.stop_client.requests) == 1
    assert node.switch_client.requests == []


def test_activation_first_target_uses_new_executor_measurement_and_waits_for_ack(fake_ros):
    node = real_runtime.make_node(configuration(), allow_execution=True)
    node.validate_scene = lambda: None
    node.validate_joint_goal = lambda start, goal: None
    node.kinematics = NS(forward=lambda q: source_sample(1.)['world_T_tcp'])
    measured = np.linspace(.01, .07, 7)
    published = node.targets.publish

    def publish(message):
        published(message)
        node.controller = (executor_state(state='TRACKING', last_sequence=message.sequence,
                                           positions=measured.tolist()), fake_ros['mono'])

    def call(client, request, **_kwargs):
        if client is node.parameters_client:
            if request.names[0] == 'enabled':
                return NS(values=[NS(type=1, bool_value=True) for _ in range(3)])
            return NS(values=[NS(type=3, double_value=.1),
                NS(type=8, double_array_value=[.2] * 7), NS(type=3, double_value=.003)])
        if client is node.list_client:
            return NS(controller=[NS(name='left_fr3_arm_controller', state='active'),
                NS(name='left_insertion_controller', state='inactive')])
        if client is node.switch_client:
            node.controller = (executor_state(state='HOLDING', last_sequence=0,
                positions=measured.tolist()), fake_ros['mono'])
            return NS(ok=True)
        raise AssertionError('Unexpected service call')

    node.targets.publish = publish
    node.call = call
    node.activate(np.zeros(7))
    assert node.owner and not node.ownership_uncertain
    assert len(node.targets.messages) == 1
    first = node.targets.messages[0]
    assert first.sequence == 1 and first.session == 17
    np.testing.assert_array_equal(first.positions, measured)
    np.testing.assert_array_equal(node.q_reference, measured)
    assert node.controller[0].last_sequence >= first.sequence


def test_published_joint_step_uses_actual_command_time_interval(fake_ros):
    node = real_runtime.make_node(configuration(), allow_execution=True)
    node.controller_session = 17
    node.controller = (executor_state(), fake_ros['mono'])
    node.executor_limits = dict(timeout=.1, speed=np.full(7, .1), stopped=.003)
    node.publish_target(np.zeros(7))
    fake_ros.update(mono=1.005, ros=100.005)
    # At 100 Hz nominal period this would be an allowed .001-rad step, but
    # only half of that wall interval has actually elapsed since transmission.
    with pytest.raises(RuntimeError, match='per-stamp velocity'):
        node.publish_target(np.full(7, .0008))
    assert len(node.targets.messages) == 1


def executor_state(**changes):
    values = dict(state='TRACKING', session=17, reason='', last_sequence=3,
                  positions=[0.] * 7, velocities=[0.] * 7,
                  stamp=NS(sec=100, nanosec=0))
    values.update(changes)
    return NS(**values)


@pytest.mark.parametrize('phase,reason,velocity,stale,success', [
    ('STOPPING', 'cancelled', 0., False, False),
    ('STOPPED', 'cancelled', 0., False, True),
    ('STOPPED', 'cancelled', .01, False, False),
    ('STOPPED', 'cancelled', 0., True, False),
    ('STOPPED', 'tracking_error', 0., False, False),
])
def test_estimated_success_waits_for_fresh_executor_stopped_hold(
        fake_ros, phase, reason, velocity, stale, success):
    node = real_runtime.make_node(configuration(), allow_execution=True)
    node.owner = True
    node.controller_session = 17
    node.executor_limits = dict(timeout=.1, speed=np.full(7, .2), stopped=.003)
    node.session.policy.insertion_success = True
    node.session.policy.state = 'inserted_unretained'
    node.controller = (executor_state(state=phase, reason=reason, velocities=[velocity] * 7),
                       fake_ros['mono'] - (.2 if stale else 0.))
    node.stop_executor()
    result = node.snapshot()
    assert result['insertion_success'] is success
    assert result['physical_success_verified'] is False
    assert result['stop_acknowledged'] is success
    assert result['no_auto_reactivate'] is True
    assert result['outcome'] == ('estimated_reached' if success else None)


@pytest.mark.parametrize('failure', ['stale_gripper', 'overload'])
def test_successful_stopped_hold_revokes_success_when_observation_fails(fake_ros, failure):
    node = real_runtime.make_node(configuration(), allow_execution=True)
    node.owner = True
    node.controller_session = 17
    node.executor_limits = dict(timeout=.1, speed=np.full(7, .2), stopped=.003)
    node.session.policy.insertion_success = True
    node.session.policy.state = 'inserted_unretained'
    node.controller = (executor_state(state='STOPPED', reason='cancelled'), fake_ros['mono'])
    node.stop_executor()
    assert node.snapshot()['insertion_success'] is True
    bad = source_sample(1., **({'gripper_time_s': .5} if failure == 'stale_gripper'
                              else {'axial_force': 6.}))
    node.sample = lambda: (np.zeros(7), bad)
    node.tick()
    published = json.loads(node.publisher.messages[-1].data)
    assert not published['insertion_success']
    assert published['outcome'] is None
    assert published['state'] == 'blocked'
    assert published['physical_success_verified'] is False
    assert node.executor_failure is not None
    assert not node.running
    assert len(node.stop_client.requests) == 1


def test_read_only_sample_exception_invalidates_previously_valid_diagnostics(fake_ros):
    node = real_runtime.make_node(configuration(), allow_execution=False)
    assert node.session.observe(1., source_sample(1.))['observation_valid']

    def unavailable_sample():
        raise RuntimeError('injected robot transport failure')

    node.sample = unavailable_sample
    node.tick()
    published = json.loads(node.publisher.messages[-1].data)
    assert published['observation']['observation_valid'] is False
    assert published['observation']['feedback_available'] is False
    assert published['observation']['invalid_reason'] == 'injected robot transport failure'
    assert node.targets.messages == []
    assert node.stop_client.requests == []


def test_alignment_to_insertion_preserves_previously_approved_target(fake_ros):
    node = real_runtime.make_node(configuration(), allow_execution=True)
    measured = source_sample(1.)
    approved = measured['world_T_tcp'].copy()
    approved[0, 3] += .0002  # Model/measurement residual must not become a target discontinuity.
    node.session.goal = approved.copy()
    node.session.policy.state = 'aligned'
    node.owner = True
    node.controller_session = 17
    node.controller = (executor_state(), fake_ros['mono'])
    node.q_reference = np.zeros(7)
    node.sample = lambda: (np.zeros(7), measured)
    node.right_check = lambda ready: None
    response = node.command('start', NS())
    assert response.success
    assert node.running and node.session.policy.state == 'feedback_advance'
    np.testing.assert_array_equal(node.session.goal, approved)
    assert node.targets.messages == []


def pose_message(matrix):
    w, x, y, z = mat2quat(matrix[:3, :3])
    return NS(position=NS(x=matrix[0, 3], y=matrix[1, 3], z=matrix[2, 3]),
              orientation=NS(w=w, x=x, y=y, z=z))


def robot_message(node, clock):
    tcp = source_sample(clock['mono'])['world_T_tcp']
    header = NS(stamp=NS(sec=int(clock['ros']), nanosec=0), frame_id='left_fr3_link0')
    zeros = NS(x=0., y=0., z=0.)
    message = NS(time=1., header=header,
        measured_joint_state=NS(name=node.names, position=[0.] * 7, velocity=[0.] * 7),
        o_t_ee=NS(pose=pose_message(tcp)), ee_t_k=NS(pose=pose_message(np.eye(4))),
        o_f_ext_hat_k=NS(wrench=NS(force=zeros, torque=zeros)),
        robot_mode=1, ROBOT_MODE_IDLE=1, ROBOT_MODE_MOVE=2,
        current_errors=NS(get_fields_and_field_types=lambda: {}))
    gripper = NS(header=copy.deepcopy(header),
        name=['left_fr3_finger_joint1', 'left_fr3_finger_joint2'], position=[.01, .01])
    gripper.header.frame_id = ''
    node.kinematics = NS(forward=lambda q, jacobian: (tcp, np.zeros((6, 7))))
    return message, gripper


def test_cached_source_timestamps_do_not_move_backwards_on_repeated_reads(fake_ros):
    node = real_runtime.make_node(configuration())
    robot, gripper = robot_message(node, fake_ros)
    node.receive_robot(robot)
    callback = node.subscriptions['/left_franka_gripper/joint_states']
    callback(gripper)
    _, first = node.sample()
    fake_ros.update(mono=1.01, ros=100.01)
    _, second = node.sample()
    assert second['time_s'] == pytest.approx(first['time_s'], abs=1.e-9)
    assert second['gripper_time_s'] == pytest.approx(first['gripper_time_s'], abs=1.e-9)
    assert node.session.observe(1., first)['observation_valid']
    assert node.session.observe(1.01, second)['observation_valid']


def test_duplicate_robot_messages_cannot_refresh_source_age(fake_ros):
    node = real_runtime.make_node(configuration())
    robot, _ = robot_message(node, fake_ros)
    node.receive_robot(robot)
    previous = node.robot
    fake_ros.update(mono=1.02, ros=100.02)
    node.receive_robot(robot)
    assert node.robot is previous


def test_frequently_read_source_keeps_original_mapping_across_cache_turnover(fake_ros):
    node = real_runtime.make_node(configuration())
    cached = NS(header=NS(stamp=NS(sec=100, nanosec=0), frame_id='cached_gripper'))
    original = node.message_time(cached, 1.)
    for index in range(1, 300):
        # Small clock-mapping drift must not re-date an already seen packet.
        fake_ros.update(mono=1. + index * .0001, ros=100. + index * .00009)
        message = NS(header=NS(stamp=NS(sec=100, nanosec=index * 90000),
                              frame_id='robot'))
        node.message_time(message, fake_ros['mono'])
        assert node.message_time(cached, 1.) == original


def right_feedback(node, now=1.):
    names = [f'right_fr3_joint{i}' for i in range(1, 8)]
    fingers = [f'right_fr3_finger_joint{i}' for i in (1, 2)]
    header = NS(frame_id='right_fr3_link0', stamp=NS(sec=100, nanosec=0))
    arm = NS(header=copy.deepcopy(header), name=names, position=[0.] * 7, velocity=[0.] * 7)
    gripper = NS(header=copy.deepcopy(header), name=fingers, position=[.035, .035])
    gripper.header.frame_id = 'right_fr3_hand'
    node.right_robot, node.right_gripper = (arm, now), (gripper, now)
    all_names = node.names + names + fingers
    aggregate = NS(header=copy.deepcopy(header), name=all_names,
                   position=[0.] * 14 + [.035, .035], velocity=[0.] * 16)
    aggregate.header.frame_id = 'aggregate'
    node.all_joints = (aggregate, now)
    return arm, gripper, aggregate


@pytest.mark.parametrize('bad_source', ['missing_arm', 'missing_gripper', 'stale_arm', 'stale_gripper'])
@pytest.mark.parametrize('operation', ['right_check', 'collision'])
def test_fresh_aggregate_cannot_hide_missing_or_stale_right_sources(fake_ros, bad_source, operation):
    node = real_runtime.make_node(configuration())
    arm, gripper, _ = right_feedback(node)
    if bad_source == 'missing_arm':
        node.right_robot = None
    elif bad_source == 'missing_gripper':
        node.right_gripper = None
    elif bad_source == 'stale_arm':
        arm.header.stamp.sec = 99
    else:
        gripper.header.stamp.sec = 99
    with pytest.raises(RuntimeError, match='Independent right'):
        if operation == 'right_check':
            node.right_check(ready=True)
        else:
            node.validate_joint_goal(np.zeros(7), np.zeros(7))
    assert node.validity_client.requests == []


def test_collision_rejects_aggregate_disagreement_with_independent_right_position(fake_ros):
    node = real_runtime.make_node(configuration())
    arm, _, _ = right_feedback(node)
    arm.position[0] = .005  # Within ready tolerance, outside aggregate consistency tolerance.
    with pytest.raises(RuntimeError, match='disagrees'):
        node.validate_joint_goal(np.zeros(7), np.zeros(7))
    assert node.validity_client.requests == []


def test_independent_fresh_right_feedback_can_establish_ready_state(fake_ros):
    node = real_runtime.make_node(configuration())
    right_feedback(node)
    node.right_check(ready=True)


@pytest.mark.parametrize('change', ['none', 'socket_pose', 'plug_pose', 'plug_frame', 'duplicate_plug'])
def test_scene_admission_checks_calibrated_socket_and_attached_plug(fake_ros, change):
    node = real_runtime.make_node(configuration())
    socket = NS(id='usb_socket', header=NS(frame_id='world'), meshes=[NS()],
        primitives=[], mesh_poses=[pose_message(np.eye(4))], primitive_poses=[])
    plug = NS(id='usb_cable_demo_plug', header=NS(frame_id='left_fr3_hand_tcp'), meshes=[NS()],
        primitives=[], mesh_poses=[pose_message(np.eye(4))], primitive_poses=[])
    scene = NS(world=NS(collision_objects=[socket]), robot_state=NS(
        attached_collision_objects=[NS(link_name='left_fr3_hand_tcp', object=plug)]))
    if change == 'socket_pose':
        socket.mesh_poses[0].position.x += .001
    elif change == 'plug_pose':
        plug.mesh_poses[0].position.x += .001
    elif change == 'plug_frame':
        plug.header.frame_id = 'world'
    elif change == 'duplicate_plug':
        scene.world.collision_objects.append(copy.deepcopy(plug))
    node.call = lambda client, request: NS(scene=scene)
    if change == 'none':
        node.validate_scene()
    else:
        with pytest.raises(RuntimeError):
            node.validate_scene()


@pytest.mark.parametrize('quaternion', [(0., 0., 0., 0.), (float('nan'), 0., 0., 1.)])
def test_robot_pose_rejects_invalid_quaternion(quaternion):
    pose = pose_message(np.eye(4))
    pose.orientation = NS(**dict(zip(('w', 'x', 'y', 'z'), quaternion)))
    with pytest.raises(ValueError, match='quaternion'):
        real_runtime.pose_matrix(pose)
