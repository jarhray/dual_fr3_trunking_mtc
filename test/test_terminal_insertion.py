"""Orchestration contracts, not physical insertion acceptance."""
from types import SimpleNamespace
import pytest
from dual_fr3_trunking_mtc.insertion import TerminalInsertion
from dual_fr3_trunking_mtc.mtc import task_builder


@pytest.mark.parametrize('execution_ok', [True, False])
def test_planning_retries_but_never_replays_failed_execution(monkeypatch, execution_ok):
    calls = []
    class Task:
        def __init__(self):
            self.solutions = []
            calls.append('new_measured_task')
        def loadRobotModel(self, node): pass
        def add(self, stage): pass
        def plan(self, count):
            calls.append('plan')
            if calls.count('plan') == 3:
                self.solutions = ['solution']
                return True
            return False
        def execute(self, solution):
            calls.append('execute')
            return execution_ok
    class Move:
        def __init__(self, *args): pass
        def setGoal(self, target): pass
        def setCostTerm(self, callback): calls.append('path_length_gate')
    monkeypatch.setattr(task_builder, 'import_mtc_modules', lambda: (None,
        SimpleNamespace(Task=Task), SimpleNamespace(CurrentState=lambda name: name, MoveTo=Move)))
    monkeypatch.setattr(task_builder, 'create_motion_planners', lambda *args: (None, None, None))
    obj = TerminalInsertion.__new__(TerminalInsertion)
    obj.node = None
    obj.config = dict(planning_attempts=10)
    obj.logger = SimpleNamespace(info=lambda *args: None)
    if execution_ok:
        obj.motion('approach', 'left_fr3_arm', 'ready')
    else:
        with pytest.raises(RuntimeError, match='execution failed'):
            obj.motion('approach', 'left_fr3_arm', 'ready')
    assert calls.count('new_measured_task') == 3
    assert calls.count('plan') == 3
    assert calls.count('execute') == 1
    assert calls.count('path_length_gate') == 3


@pytest.mark.parametrize('reject', [None, 'recoverable_alignment', 'right_return_ready', 'alignment', 'insertion_collision_preflight'])
def test_cached_continuation_is_validated_without_replanning(monkeypatch, reject):
    import logging
    import numpy as np
    from dual_fr3_maniskill.cable.model import USB_LINK
    from dual_fr3_maniskill.usb.geometry import HOLE, TIP, USB_IN_SOCKET, SOCKET_NAME
    from dual_fr3_maniskill.usb.insertion import InsertionLimits
    from dual_fr3_trunking_mtc.mtc import cached_execution

    names = ['withdraw_right', 'right_return_ready', 'socket_preinsert', 'insertion_collision_preflight']
    motions = {name: object() for name in names}
    opening, planned = object(), object()
    events = []
    monkeypatch.setattr(cached_execution, 'selected_stage_solution',
                        lambda selected, name: opening if selected is planned and name == 'preview_right_release' else None)
    monkeypatch.setattr(cached_execution, 'cache_selected_stages', lambda selected: [
        SimpleNamespace(spec=SimpleNamespace(name=name), solution=motions[name]) for name in names])
    usb = np.eye(4)
    usb[:3, :3] = USB_IN_SOCKET
    usb[:3, 3] = HOLE + [InsertionLimits().preinsert_m, 0., 0.] - USB_IN_SOCKET @ TIP
    if reject == 'alignment':
        usb[1, 3] += .01
    if reject == 'recoverable_alignment':
        usb[1, 3] += .001  # inside servo capture, outside direct insertion gate
    scene = SimpleNamespace(
        get_frame_transform=lambda frame: usb if frame == USB_LINK else np.eye(4),
        allowed_collision_matrix=SimpleNamespace(set_entry=lambda *a: events.append(a)),
    )

    def validate(name, solution, *, gripper=False):
        assert solution is (opening if gripper else motions[name])
        events.append(name)
        return name != reject

    obj = TerminalInsertion.__new__(TerminalInsertion)
    obj.config, obj.logger = {}, logging.getLogger(__name__)
    obj.approach_plan = SimpleNamespace(planned=planned)
    assert obj.validate_cached_continuation(SimpleNamespace(scene=scene, validate=validate)) is (reject in (None, 'recoverable_alignment'))
    prefix = ['preview_right_release', 'withdraw_right', 'right_return_ready']
    if reject != 'right_return_ready':
        prefix.append('socket_preinsert')
        if reject != 'alignment':
            prefix.extend([(USB_LINK, SOCKET_NAME, True), 'insertion_collision_preflight'])
    assert events == prefix


def test_socket_installation_preserves_existing_collision_permissions():
    from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry
    acm = AllowedCollisionMatrix(entry_names=['plate','trunking','left_fr3_hand','usb_cable_demo_plug'],
        entry_values=[AllowedCollisionEntry(enabled=row) for row in
                      [[False,True,False,False],[True,False,False,False],
                       [False,False,False,True],[False,False,True,False]]])
    obj = TerminalInsertion.__new__(TerminalInsertion)
    obj.config = {}
    obj.get_scene = None
    obj.client = SimpleNamespace(_call=lambda *a, **kw: SimpleNamespace(scene=SimpleNamespace(allowed_collision_matrix=acm)))
    obj.command = lambda op: {'socket_world_pose':{'position_m':[.545,.280,.010], 'quaternion_wxyz':[1.,0.,0.,0.]}}
    scenes = []
    obj.apply = scenes.append
    obj.sync_socket()
    result = scenes[0].allowed_collision_matrix
    def allowed(a,b):
        return result.entry_values[result.entry_names.index(a)].enabled[result.entry_names.index(b)]
    assert allowed('plate','trunking')
    assert allowed('left_fr3_hand','usb_cable_demo_plug')
    for fixture in ('plate','trunking'):
        assert allowed('usb_socket',fixture) and allowed(fixture,'usb_socket')
    for body in ('left_fr3_hand','usb_cable_demo_plug'):
        assert not allowed('usb_socket',body) and not allowed(body,'usb_socket')
    assert scenes[0].world.collision_objects[0].mesh_poses[0].position.z == .010


@pytest.mark.parametrize('failure', [None, 'open_right', 'right_return_ready', 'retained'])
def test_right_returns_before_insertion_and_left_releases_only_after_retention(failure):
    events = []
    obj = TerminalInsertion.__new__(TerminalInsertion)
    obj.config = {}; obj.saved_acm = None
    obj.logger = SimpleNamespace(info=lambda *a: None, error=lambda *a: None, exception=lambda *a: None)
    pose = dict(position_m=[0.,0.,0.], quaternion_wxyz=[1.,0.,0.,0.])
    def command(op):
        events.append(op)
        if op == failure: raise RuntimeError('injected failure')
        if op == 'status': return dict(return_complete=False, state='not_started')
        if op == 'target': return dict(**pose, end_tcp_pose=pose)
        if op == 'heartbeat':
            return dict(state='retained' if 'start' in events else 'aligned', insertion_success='start' in events)
        return {}
    obj.command = command
    def motion(name, *args, **kwargs):
        events.append(name)
        if name == failure: raise RuntimeError('injected execution failure')
    obj.motion = motion
    obj.approach_plan = SimpleNamespace(execute=motion)
    obj.withdraw = lambda side: events.append('withdraw_'+side)
    obj.detach_measured_usb = lambda status: events.append('detach_usb')
    def gripper(request):
        name = 'open_right' if request.actor == 'follower' else 'open_left'
        events.append(name)
        return name != failure
    result = obj.execute(SimpleNamespace(execute=gripper))
    assert result is (failure is None)
    if failure in ('open_right','right_return_ready'):
        assert 'target' not in events and 'start' not in events and 'open_left' not in events
        assert ('fail_right_release' if failure == 'open_right' else 'fail_right_return') in events
    else:
        assert events.index('open_right') < events.index('withdraw_right') < events.index('right_return_ready')
        assert events.index('right_returned') < events.index('target') < events.index('start')
        assert events.index('socket_preinsert') < events.index('align') < events.index('start')
        if failure == 'retained':
            assert 'open_left' not in events
        else:
            assert events.index('retained') < events.index('open_left') < events.index('withdraw_left') < events.index('left_return_ready') < events.index('returned')
            assert events.count('open_right') == events.count('right_return_ready') == 1


@pytest.mark.parametrize('failure', ['detach', 'open_left', 'released', 'withdraw_left',
                                   'restore', 'left_return_ready', 'returned'])
def test_left_failure_reports_phase_cancels_and_restores_permissions(failure):
    from moveit_msgs.msg import AllowedCollisionMatrix

    events = []
    obj = TerminalInsertion.__new__(TerminalInsertion)
    obj.config = {}
    obj.saved_acm = None
    saved = AllowedCollisionMatrix()
    obj.logger = SimpleNamespace(info=lambda *a: None, error=lambda *a: None, exception=lambda *a: None)

    def event(name):
        events.append(name)
        if name == failure:
            raise RuntimeError('injected ' + name)

    def command(operation):
        event(operation)
        return dict(return_complete=False, state='not_started')

    obj.command = command
    obj.release_right = lambda controller: None
    obj.approach_plan = object()
    obj.return_right = lambda: None
    obj.align_left = lambda: None
    obj.feedback_insert = lambda: None

    def detach(status):
        obj.saved_acm = saved
        event('detach')

    obj.detach_measured_usb = detach
    obj.open_gripper = lambda controller, side: event('open_' + side)
    obj.withdraw = lambda side: event('withdraw_' + side)
    obj.motion = lambda name, *args: event(name)

    def apply(scene):
        assert scene.allowed_collision_matrix == saved
        event('restore')

    obj.apply = apply
    assert not obj.execute(None)
    assert events.index('retained') < events.index('detach')
    report = 'fail_release' if failure in ('detach', 'open_left', 'released') else 'fail_return'
    assert events.index(report) < events.index('cancel')
    assert 'restore' in events
    if failure in ('detach', 'open_left', 'released', 'withdraw_left', 'restore'):
        assert events[-1] == 'restore'  # finally retries cleanup even after failure
    else:
        assert events.index('restore') < events.index('left_return_ready')
        assert obj.saved_acm is None


@pytest.mark.parametrize('option,absent', [
    ('retain_after_success', 'retained'),
    ('release_after_retention', 'release_left'),
    ('return_after_release', 'return_left'),
])
def test_optional_finish_stages_preserve_cancel_and_order(option, absent):
    obj = TerminalInsertion.__new__(TerminalInsertion)
    obj.config = {option: False}
    obj.saved_acm = None
    events = []
    obj.command = lambda op: (events.append(op) or dict(return_complete=False, state='not_started'))
    obj.release_right = lambda controller: events.append('release_right')
    obj.approach_plan = object()
    obj.return_right = lambda: events.append('return_right')
    obj.align_left = lambda: events.append('align_left')
    obj.feedback_insert = lambda: events.append('feedback_insert')
    obj.release_left = lambda controller, status: events.append('release_left')
    obj.return_left = lambda: events.append('return_left')
    assert obj.execute(None)
    assert events[:5] == ['status', 'release_right', 'return_right', 'align_left', 'feedback_insert']
    assert absent not in events and events[-1] == 'cancel'


def test_failed_local_alignment_never_starts_insertion_or_releases_left():
    obj = TerminalInsertion.__new__(TerminalInsertion)
    obj.config, obj.saved_acm = {}, None
    events = []
    obj.logger = SimpleNamespace(info=lambda *a: None, error=lambda *a: None, exception=lambda *a: None)
    obj.approach_plan = SimpleNamespace(execute=events.append)
    obj.release_right = lambda _: events.append('release_right')
    obj.return_right = lambda: events.append('return_right')
    def command(op):
        events.append(op)
        if op == 'status': return dict(state='not_started', return_complete=False)
        if op == 'heartbeat': return dict(state='blocked', reason='local_path_collision')
        return {}
    obj.command = command
    assert not obj.execute(None)
    assert events == ['status', 'release_right', 'return_right', 'target',
                      'socket_preinsert', 'align', 'heartbeat', 'cancel']
