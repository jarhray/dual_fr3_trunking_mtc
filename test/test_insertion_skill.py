"""Skill orchestration and measured-grasp geometry, without a physics engine."""
from types import SimpleNamespace as NS

import numpy as np
import pytest
from transforms3d.quaternions import quat2mat

from dual_fr3_maniskill.insertion import InsertionLimits
from dual_fr3_maniskill.insertion_geometry import HOLE, TIP, USB_IN_SOCKET
from dual_fr3_trunking_mtc.insertion_planning import approach_goals, InsertionApproach
from dual_fr3_trunking_mtc.insertion_skill import run_skill


@pytest.mark.parametrize('config', [{}, {'preinsert_m': .012, 'hole_center_m': [0., .02, .07]}])
def test_approach_goals_put_measured_usb_tip_at_requested_socket_depth(config):
    def transform(angle, position):
        result = np.eye(4)
        result[:3, :3] = quat2mat([np.cos(angle / 2), 0., 0., np.sin(angle / 2)])
        result[:3, 3] = position
        return result

    base = transform(.6, [.5, .3, .01])
    tcp = transform(-.4, [.4, .2, .1])
    grasp = transform(.03, [.001, -.002, .012])
    frames = {'usb_socket': base, 'left_fr3_hand_tcp': tcp, 'usb_cable_demo_plug': tcp @ grasp}
    scene = NS(planning_frame='world', get_frame_transform=frames.__getitem__,
               knows_frame_transform=frames.__contains__)
    limits = InsertionLimits.read(config)
    for goal, depth in zip(approach_goals(scene, config), (-limits.preinsert_m, limits.target_depth_m)):
        pose = goal.pose
        result = np.eye(4)
        q = pose.orientation
        result[:3, :3] = quat2mat([q.w, q.x, q.y, q.z])
        result[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        usb_local = np.linalg.inv(base) @ result @ grasp
        np.testing.assert_allclose(usb_local[:3, :3], USB_IN_SOCKET, atol=1e-12)
        np.testing.assert_allclose(usb_local[:3, 3] + usb_local[:3, :3] @ TIP,
                                   np.array(config.get('hole_center_m', HOLE)) + [-depth, 0., 0.], atol=1e-12)


@pytest.mark.parametrize('execute,plan_ok', [(False, True), (True, True), (True, False)])
def test_standalone_plans_before_reusing_terminal_sequence(execute, plan_ok):
    events = []
    terminal = NS(
        command=lambda op: dict(state='not_started', return_complete=False),
        plan_from_current_state=lambda: events.append('plan') or plan_ok,
        execute=lambda gripper: events.append('execute') or True,
    )
    assert run_skill(terminal, None, execute=execute) is plan_ok
    assert events == ['plan'] + (['execute'] if execute and plan_ok else [])


@pytest.mark.parametrize('state', ['complete', 'feedback_advance', 'cancelled', 'right_return_failed'])
def test_standalone_repeat_and_failed_or_busy_state_issue_no_motion(state):
    def unexpected():
        raise AssertionError('must not plan or issue motion')

    terminal = NS(command=lambda op: dict(state=state, return_complete=state == 'complete'),
                  plan_from_current_state=unexpected, execute=unexpected)
    if state == 'complete':
        assert run_skill(terminal, None, execute=True)
    else:
        with pytest.raises(RuntimeError, match='reset'):
            run_skill(terminal, None, execute=True)


def test_cached_approach_failure_does_not_replay():
    calls = []
    plan = InsertionApproach(NS(task=NS(execute=lambda value: calls.append(value) or False)),
                             {'withdraw_right': 'selected'})
    with pytest.raises(RuntimeError, match='execution failed'):
        plan.execute('withdraw_right')
    assert calls == ['selected']


def test_missing_usb_frame_is_rejected_before_planning():
    scene = NS(knows_frame_transform=lambda frame: frame != 'usb_cable_demo_plug')
    with pytest.raises(ValueError, match='Missing insertion planning frame'):
        approach_goals(scene, {})
