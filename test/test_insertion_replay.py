"""A replay must reproduce policy output and never need a robot connection."""
import copy
from pathlib import Path
import numpy as np
import pytest

from dual_fr3_trunking_mtc.insertion_task.replay import replay
from dual_fr3_trunking_mtc.insertion_task.real_session import load_real_config, RealInsertionSession
from dual_fr3_maniskill.usb.geometry import HOLE, TIP, USB_IN_SOCKET


def records():
    observation = dict(depth_m=-.008, lateral_error_m=0., orientation_error_rad=0.,
        feedback_available=True, grasp_valid=None, temporary_support=None,
        resistance_N=0., lateral_N=0., torque_Nm=0., relative_speed_m_s=0., relative_angular_rad_s=0.,
        observation_valid=True, pose_source='calibrated_estimate', calibration_id='offline_test',
        gripper_closed=True, grasp_assumption_valid=True)
    result = [dict(time_s=0., operation='begin', observation=observation)]
    for index in range(1, 10):
        sample = copy.deepcopy(observation)
        sample['depth_m'] += index * .00001
        result.append(dict(time_s=.02 * index, observation=sample))
    return result


def test_replay_identical_observations_produce_identical_policy_trace():
    config = {'insertion': {'observation_mode': 'calibrated_estimate'}}
    first = list(replay(config, records()))
    assert first == list(replay(config, records()))
    assert first[-1]['speed_m_s'] > 0
    assert not first[-1]['physical_success_verified']


def test_replay_rejects_time_reversal_and_missing_begin():
    config = {'insertion': {'observation_mode': 'calibrated_estimate'}}
    sequence = records()
    sequence[-1]['time_s'] = 0.
    with pytest.raises(ValueError, match='strictly increase'):
        list(replay(config, sequence))
    with pytest.raises(ValueError, match='explicit begin'):
        list(replay(config, records()[1:]))


def test_recorded_real_session_alignment_and_insertion_replay_identically():
    config = load_real_config(Path(__file__).parents[1] / 'config/real_insertion.example.yaml')
    calibration = config['insertion']['calibration']
    calibration.update(verified=True, grasp_assumption_valid=True,
                       uncertainty_translation_m=0., uncertainty_angle_rad=0.)
    config['real_runtime']['limits_verified'] = True
    config['insertion']['speed_m_s'] = .002
    session = RealInsertionSession(config)
    events = []
    session.record_policy_event = lambda event: events.append(copy.deepcopy(event))
    base = np.asarray(calibration['world_T_socket'])
    tcp = np.eye(4)
    tcp[:3, :3] = USB_IN_SOCKET
    tcp[:3, 3] = HOLE + [.008, .00015, 0.] - USB_IN_SOCKET @ TIP
    tcp = base @ tcp
    now = 1.

    def sample():
        return dict(time_s=now, world_T_tcp=tcp, world_T_k=tcp,
            tcp_linear_velocity_world=[0.] * 3, tcp_angular_velocity_world=[0.] * 3,
            wrench_world_at_k=[0.] * 6, wrench_time_s=now, gripper_time_s=now,
            gripper_width_m=.01, gripper_healthy=True, gripper_grasped=None,
            robot_healthy=True, wrench_available=True)

    session.begin(now, sample(), align=True)
    for _ in range(100):
        now += .02
        goal = session.next_goal(now, sample())
        if goal is not None:
            session.accept_goal(goal)
            tcp = goal
        if session.policy.state == 'aligned':
            break
    assert session.policy.state == 'aligned'
    assert session.policy.alignment_travel > 0
    now += .02
    session.begin(now, sample(), align=False)
    for _ in range(1000):
        now += .02
        goal = session.next_goal(now, sample())
        if goal is None:
            break
        session.accept_goal(goal)
        tcp = goal
    assert session.policy.insertion_success
    events.append(dict(kind='runtime_snapshot', monotonic_time_s=now))
    actual = list(replay(config, events))[-1]
    assert {key: actual[key] for key in session.policy.snapshot()} == session.policy.snapshot()
