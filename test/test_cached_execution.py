import logging
from types import SimpleNamespace as NS

import pytest
from geometry_msgs.msg import Pose
from moveit_msgs.msg import MoveItErrorCodes

from dual_fr3_trunking_mtc.mtc import cached_execution, planning
from dual_fr3_trunking_mtc.mtc.planning import PlannedTask


LOGGER = logging.getLogger(__name__)


def spec(index, kind="MoveTo"):
    return NS(
        stage_index=index, name=f"stage_{index}", executable=True,
        mtc_stage_type=kind, confirmation_required=False, ik_frame="tcp",
        actor="leader", gripper_profile="cable_tip", gripper_action="",
        gripper_width_override=None,
    )


def args(**kwargs):
    return NS(**dict(
        {"planning_attempts": 3, "execution_replan_attempts": 2, "publish_solution": False},
        **kwargs,
    ))


def message(*ids):
    return NS(sub_solution=[], sub_trajectory=[NS(info=NS(id=value)) for value in ids])


class Solution:
    def __init__(self, *ids, target_x=1.0):
        self.ids = ids
        pose = Pose()
        pose.position.x = target_x
        pose.orientation.w = 1.0
        self.end = NS(scene=NS(
            planning_frame="world",
            current_state=NS(update=lambda: None, get_pose=lambda _: pose),
        ))

    def toMsg(self, _introspection):
        return message(*self.ids)


class Result:
    def __init__(self, code):
        self.val = code

    def __bool__(self):
        return self.val == MoveItErrorCodes.SUCCESS


class Task:
    def __init__(self, candidates, results=()):
        self.candidates = candidates
        self.results = iter(results)
        self.executed = []

    def introspection(self):
        return object()

    def __getitem__(self, name):
        return NS(solutions=self.candidates[name])

    def execute(self, solution):
        self.executed.append(solution)
        value = next(self.results, MoveItErrorCodes.SUCCESS)
        if isinstance(value, Exception):
            raise value
        return Result(value)


def planned(specs, results=(), offset=10):
    selected = [Solution(offset + i, target_x=float(i + 1)) for i in range(len(specs))]
    task = Task({s.name: [candidate] for s, candidate in zip(specs, selected)}, results)
    return PlannedTask(task, Solution(*(s.ids[0] for s in selected)), tuple(specs))


def unexpected_plan(*_args, **_kwargs):
    raise AssertionError("successful cached execution must not call the planner")


def test_success_executes_connected_selected_solutions_without_replanning():
    specs = [spec(0), spec(1)]
    first, second = Solution(11), Solution(22)
    task = Task({"stage_0": [Solution(10), first], "stage_1": [second, Solution(23)]})
    selected = PlannedTask(task, Solution(11, 22), tuple(specs))

    assert cached_execution.execute_cached_solution(
        selected, None, None, args(), LOGGER, planner=unexpected_plan,
    )
    assert task.executed == [first, second]


def test_missing_candidate_stops_before_executing_anything():
    selected = planned([spec(0), spec(1)])
    selected.task.candidates["stage_1"] = [Solution(999)]
    assert not cached_execution.execute_cached_solution(
        selected, None, None, args(), LOGGER, planner=unexpected_plan,
    )
    assert selected.task.executed == []


def test_recovery_replans_only_unfinished_stages_to_original_relative_targets():
    specs = [spec(0), spec(1, "MoveRelative"), spec(2, "MoveRelative")]
    initial = planned(specs, [MoveItErrorCodes.SUCCESS, MoveItErrorCodes.CONTROL_FAILED])
    replacement = planned(specs[1:], offset=100)
    calls = []

    def replan(node, task_plan, remaining, parsed_args, logger, **kwargs):
        calls.append((remaining, kwargs["recovery_pose_goals"]))
        return replacement

    assert cached_execution.execute_cached_solution(
        initial, None, None, args(), LOGGER, planner=replan,
    )
    assert len(initial.task.executed) == 2
    assert len(replacement.task.executed) == 2
    assert calls[0][0] == tuple(specs[1:])
    assert calls[0][1]["stage_1"].pose.position.x == 2.0
    assert calls[0][1]["stage_2"].pose.position.x == 3.0


def test_recovery_budget_is_bounded_and_original_goal_never_drifts():
    specs = [spec(0, "MoveRelative")]
    initial = planned(specs, [MoveItErrorCodes.CONTROL_FAILED])
    calls = []

    def replan(*_args, **kwargs):
        calls.append(kwargs["recovery_pose_goals"]["stage_0"].pose.position.x)
        replacement = planned(specs, [MoveItErrorCodes.CONTROL_FAILED], offset=100)
        replacement.task.candidates["stage_0"][0].end.scene.current_state.get_pose = (
            lambda _: Pose()
        )
        return replacement

    assert not cached_execution.execute_cached_solution(
        initial, None, None, args(execution_replan_attempts=2), LOGGER, planner=replan,
    )
    assert calls == [1.0, 1.0]


def test_preparation_recovery_keeps_chosen_joint_goals_and_does_not_repeat_confirmation():
    specs = [spec(0), spec(1)]
    specs[0].confirmation_required = True
    initial = planned(specs, [MoveItErrorCodes.CONTROL_FAILED])
    initial.preparation_joint_goals = {"stage_0": {"arm_joint": 0.7}}
    replacement = planned(specs, offset=100)
    confirmations, calls = [], []

    def replan(*_args, **kwargs):
        calls.append(kwargs["preparation_joint_goals"])
        return replacement

    assert cached_execution.execute_cached_solution(
        initial, None, None, args(), LOGGER, planner=replan,
        confirmation_callback=lambda spec: confirmations.append(spec.stage_index) or True,
    )
    assert calls == [{"stage_0": {"arm_joint": 0.7}}]
    assert confirmations == [0]
    assert len(initial.task.executed) == 1
    assert len(replacement.task.executed) == 2


@pytest.mark.parametrize("result", [
    MoveItErrorCodes.PREEMPTED, MoveItErrorCodes.FAILURE, RuntimeError("connection lost"),
])
def test_cancellation_or_unknown_execution_state_never_restarts_motion(result):
    initial = planned([spec(0)], [result])
    assert not cached_execution.execute_cached_solution(
        initial, None, None, args(), LOGGER, planner=unexpected_plan,
    )


def test_replanning_exhausted_does_not_execute_old_trajectory_again():
    initial = planned([spec(0)], [MoveItErrorCodes.INVALID_MOTION_PLAN])
    assert not cached_execution.execute_cached_solution(
        initial, None, None, args(), LOGGER, planner=lambda *a, **k: None,
    )
    assert len(initial.task.executed) == 1


def test_gripper_is_not_replayed_when_a_later_motion_needs_recovery():
    specs = [spec(0, "GripperOperation"), spec(1)]
    initial = planned(specs, [MoveItErrorCodes.CONTROL_FAILED])
    replacement = planned(specs[1:], offset=100)
    gripper_calls = []
    gripper = NS(execute=lambda request: gripper_calls.append(request) or True)
    assert cached_execution.execute_cached_solution(
        initial, None, None, args(), LOGGER, gripper_controller=gripper,
        planner=lambda *a, **k: replacement,
    )
    assert len(gripper_calls) == 1


def test_gripper_failure_does_not_continue_or_replan():
    initial = planned([spec(0, "GripperOperation"), spec(1)])
    assert not cached_execution.execute_cached_solution(
        initial, None, None, args(), LOGGER,
        gripper_controller=NS(execute=lambda request: False), planner=unexpected_plan,
    )
    assert not initial.task.executed


@pytest.mark.parametrize("failed", ["spawn", "left", "right", "release_verify", None])
def test_contact_preparation_failure_prevents_later_stages_and_transport(failed):
    specs = [spec(0, "SimulationCable"), spec(1, "GripperOperation"),
             spec(2, "GripperOperation"), spec(3, "SimulationCable"), spec(4)]
    specs[0].cable_operation, specs[3].cable_operation = "spawn", "release_verify"
    specs[1].actor, specs[2].actor = "left", "right"
    specs[-1].phase = "formal"
    initial = planned(specs)
    events = []

    def close(request):
        events.append(request.actor)
        return failed != request.actor

    def insert(_spec):
        events.append(_spec.cable_operation)
        return failed != _spec.cable_operation

    result = cached_execution.execute_cached_solution(
        initial, None, None, args(), LOGGER, gripper_controller=NS(execute=close),
        cable_controller=NS(execute=insert, ensure_grasp=lambda: True), planner=unexpected_plan,
    )
    assert result is (failed is None)
    all_events = ["spawn", "left", "right", "release_verify"]
    assert events == (all_events if failed is None else all_events[:all_events.index(failed)+1])
    assert len(initial.task.executed) == int(failed is None)


def test_lost_grasp_blocks_formal_transport_without_motion_or_recovery():
    transport = spec(0)
    transport.phase = "formal"
    initial = planned([transport])
    def failed_status():
        raise RuntimeError("dropped")
    assert not cached_execution.execute_cached_solution(initial, None, None, args(), LOGGER,
        cable_controller=NS(ensure_grasp=failed_status), planner=unexpected_plan)
    assert not initial.task.executed


def test_descent_recovery_does_not_reinsert_cable():
    specs = [spec(0, "SimulationCable"), spec(1)]
    initial = planned(specs, [MoveItErrorCodes.CONTROL_FAILED])
    replacement = planned(specs[1:], offset=100)
    calls = []
    assert cached_execution.execute_cached_solution(
        initial, None, None, args(), LOGGER,
        cable_controller=NS(execute=lambda spec: calls.append(spec) or True),
        planner=lambda *a, **kw: replacement,
    )
    assert calls == [specs[0]]


def test_planning_retries_preserve_first_success_and_stop(monkeypatch):
    solution = object()
    tasks = [NS(plan=lambda: False, solutions=[]), NS(plan=lambda: True, solutions=[solution])]
    built = []

    def build(*_args, **_kwargs):
        task = tasks[len(built)]
        built.append(task)
        return task, []

    monkeypatch.setattr(planning, "create_task_from_args", build)
    result = planning.plan_with_retries(None, None, [spec(0)], args(), LOGGER)
    assert result.task is tasks[1]
    assert result.solution is solution
    assert len(built) == 2


def test_planning_failure_retries_are_bounded(monkeypatch):
    built = []

    def build(*_args, **_kwargs):
        built.append(True)
        return NS(plan=lambda: False, solutions=[]), []

    monkeypatch.setattr(planning, "create_task_from_args", build)
    assert planning.plan_with_retries(None, None, [], args(planning_attempts=3), LOGGER) is None
    assert len(built) == 3


def test_planning_configuration_exception_does_not_loop(monkeypatch):
    built = []

    def build(*_args, **_kwargs):
        built.append(True)
        raise ValueError("invalid constraint parameter")

    monkeypatch.setattr(planning, "create_task_from_args", build)
    assert planning.plan_with_retries(None, None, [], args(), LOGGER) is None
    assert len(built) == 1


@pytest.mark.parametrize('replan_ok', [False, True])
def test_measured_grasp_replans_suffix_before_only_confirmation(replan_ok):
    stages = [spec(0, 'GripperOperation'), spec(1, 'SimulationCable'), spec(2)]
    stages[1].cable_operation = 'release_verify'
    stages[2].confirmation_required = True
    stages[2].primitive = 'dual_cartesian_descent'
    initial = planned(stages)
    replacement = planned(stages[2:], offset=100)
    events = []

    def replan(*positional, **kwargs):
        assert positional[2] == tuple(stages[2:])
        events.append('replan')
        return replacement if replan_ok else None

    cable = NS(execute=lambda s: events.append(s.cable_operation) or True,
               ensure_grasp=lambda: events.append('check_grasp') or True)
    result = cached_execution.execute_cached_solution(
        initial, None, None, args(), LOGGER, planner=replan, replan_after_grasp=True,
        cable_controller=cable,
        gripper_controller=NS(execute=lambda r: events.append('close') or True),
        confirmation_callback=lambda s: events.append('enter') or True,
    )
    assert result is replan_ok
    assert not initial.task.executed
    assert events[:3] == ['close', 'release_verify', 'replan']
    assert events[3:] == (['check_grasp', 'enter', 'check_grasp'] if replan_ok else [])
    assert len(replacement.task.executed) == int(replan_ok)


def test_unplannable_terminal_continuation_rejects_transport_candidate(monkeypatch):
    tasks = [NS(plan=lambda: True, solutions=[object()]) for _ in range(2)]
    built = iter(tasks)
    monkeypatch.setattr(planning, 'create_task_from_args', lambda *a, **kw: (next(built), []))
    checked = []

    def validate(candidate):
        checked.append(candidate.task)
        return len(checked) == 2

    result = planning.plan_with_retries(
        None, None, [], args(), LOGGER, solution_validator=validate)
    assert checked == tasks
    assert result.task is tasks[1]


def test_measured_replan_keeps_selected_anchor_branch_without_other_arm_joints():
    anchor = spec(0)
    anchor.primitive = 'direct_move_to_next_anchor'
    anchor.group = 'left_fr3_arm'
    group = NS(active_joint_model_names=['left_joint'])
    state = NS(joint_positions={'left_joint': .3, 'right_joint': -.7})
    scene = NS(current_state=state, robot_model=NS(get_joint_model_group=lambda name: group))
    cached = [cached_execution.CachedStage(anchor, NS(end=NS(scene=scene)))]
    targets = cached_execution.capture_anchor_joint_targets(cached)
    state.joint_positions['left_joint'] = 2.
    assert targets == {anchor.name: {'left_joint': .3}}


@pytest.mark.parametrize("preparation_enabled", [False, True])
@pytest.mark.parametrize("planning_succeeds", [False, True])
@pytest.mark.parametrize("execute_enabled", [False, True])
def test_main_prechecks_preparation_before_any_execution(
    monkeypatch, preparation_enabled, planning_succeeds, execute_enabled,
):
    from dual_fr3_trunking_mtc import mtc_prototype

    events = []
    selected = object() if planning_succeeds else None
    rclcpp = NS(
        init=lambda: None, shutdown=lambda: None,
        NodeOptions=lambda **kwargs: None, Node=lambda *a: object(),
    )
    monkeypatch.setattr(mtc_prototype, "_import_mtc_modules", lambda: (rclcpp, None, None))
    monkeypatch.setattr(mtc_prototype, "GripperController", lambda *a, **kw: NS(close=lambda: None))
    monkeypatch.setattr(mtc_prototype, "_start_stage_sequence_publisher", lambda *a: None)

    def prepare(*_args, **_kwargs):
        raise AssertionError("preparation must not execute before complete lookahead")

    def plan(_node, _task_plan, specs, *_args, **_kwargs):
        assert all(s.phase == "formal" for s in specs)
        events.append("plan")
        return selected

    def execute(result, *_args, **_kwargs):
        assert result is selected
        events.append("execute")
        return True

    def candidate_search(_node, _task_plan, specs, *_args, **_kwargs):
        assert any(s.phase == "preparation" for s in specs)
        assert any(s.phase == "formal" for s in specs)
        events.append("candidate_search")
        return selected

    monkeypatch.setattr(mtc_prototype, "_execute_stage_by_stage", prepare)
    monkeypatch.setattr(mtc_prototype, "plan_with_retries", plan)
    monkeypatch.setattr(mtc_prototype, "plan_preparation_candidates", candidate_search)
    monkeypatch.setattr(mtc_prototype, "execute_cached_solution", execute)
    code = mtc_prototype.main([
        '--simulation-backend', 'gazebo',
        "--execute", str(execute_enabled), "--preparation-enabled", str(preparation_enabled),
        "--publish-solution", "false", "--keep-alive-sec", "0",
    ])
    assert code == (0 if planning_succeeds else 2)
    expected = ["candidate_search" if preparation_enabled else "plan"]
    assert events == expected + (["execute"] if planning_succeeds and execute_enabled else [])
