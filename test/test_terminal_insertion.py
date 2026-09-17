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
        if op == 'heartbeat': return dict(state='retained', insertion_success=True)
        return {}
    obj.command = command
    def motion(name, *args, **kwargs):
        events.append(name)
        if name == failure: raise RuntimeError('injected execution failure')
    obj.motion = motion
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
        if failure == 'retained':
            assert 'open_left' not in events
        else:
            assert events.index('retained') < events.index('open_left') < events.index('withdraw_left') < events.index('left_return_ready') < events.index('returned')
            assert events.count('open_right') == events.count('right_return_ready') == 1
