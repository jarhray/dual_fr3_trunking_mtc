import copy
import logging
import math
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest
import yaml
from geometry_msgs.msg import PoseStamped

from dual_fr3_trunking_mtc.mtc import preparation_search as search
from dual_fr3_trunking_mtc.mtc_prototype import _parse_args
from dual_fr3_trunking_mtc.mtc.task_builder import rpy_to_quaternion


LOGGER = logging.getLogger(__name__)


def search_args(**overrides):
    args = _parse_args([])
    args.preparation_ik_candidates = 3
    args.preparation_candidate_attempts = 2
    args.preparation_search_timeout = 30.0
    for name, value in overrides.items():
        setattr(args, name, value)
    return args


def approach(actor, index):
    return NS(name=f"{actor}_approach", actor=actor, stage_index=index,
              executable=True, phase="preparation", primitive="move_above_initial_keypoint",
              from_index=0, target_yaw=0.0, vector=(0.0, 0.0, 0.05),
              mtc_stage_type="MoveTo", group=actor, ik_frame=f"{actor}_tcp")


def task_specs():
    return (approach("leader", 0), approach("follower", 1),
            NS(name="follow_path", stage_index=2, phase="formal", executable=True,
               primitive="cartesian_translate", mtc_stage_type="MoveRelative"))


TASK_PLAN = NS(keypoints=[NS(frame_id="base", position=(0.0, 0.0, 0.0))])


class State:
    def __init__(self):
        self.values = {"leader_joint": -1.0, "follower_joint": -1.0}

    def __copy__(self):
        state = State()
        state.values = self.values.copy()
        return state

    @property
    def joint_positions(self):
        return self.values.copy()

    @joint_positions.setter
    def joint_positions(self, values):
        self.values.update(values)

    def update(self):
        pass


def scene():
    return NS(current_state=State(), is_state_valid=lambda *_: True)


def fake_sampler(_scene, _target, spec, *_args, **_kwargs):
    return [{f"{spec.actor}_joint": 0.0}, {f"{spec.actor}_joint": 1.0}]


class Task:
    def __init__(self, success):
        self.properties = {}
        self.solutions = [object()] if success else []

    def plan(self, count):
        assert count == 1
        return bool(self.solutions)

    def __getitem__(self, _name):
        return NS(failures=[NS(comment="Cartesian continuation unavailable")])


def test_search_retains_first_complete_solution_and_fixed_joint_goals():
    snapshot = scene()
    specs = task_specs()
    calls, captures = [], []

    def factory(_node, _plan, actual_specs, _args, **kwargs):
        assert actual_specs == specs
        assert kwargs["start_scene"] is snapshot
        assert snapshot.current_state.values == State().values
        calls.append(kwargs["preparation_joint_goals"])
        task = Task(len(calls) == 2)
        return task, None

    def capture(_node):
        captures.append(True)
        return search.CapturedScene(snapshot, capture)

    planned = search.plan_preparation_candidates(
        None, TASK_PLAN, specs, search_args(), LOGGER,
        scene_provider=capture, sampler=fake_sampler, task_factory=factory,
    )
    assert len(captures) == 1
    assert len(calls) == 2
    assert planned.solution is planned.task.solutions[0]
    assert planned.specs == specs
    assert planned.model_owner is capture
    assert planned.preparation_joint_goals == {
        "leader_approach": {"leader_joint": 0.0},
        "follower_approach": {"follower_joint": 1.0},
    }
    calls[-1]["leader_approach"]["leader_joint"] = 10.0
    assert planned.preparation_joint_goals["leader_approach"]["leader_joint"] == 0.0


def test_failed_continuations_visit_all_pairs_before_retrying():
    calls = []

    def factory(*_args, **kwargs):
        calls.append(copy.deepcopy(kwargs["preparation_joint_goals"]))
        return Task(False), None

    assert search.plan_preparation_candidates(
        None, TASK_PLAN, task_specs(), search_args(), LOGGER,
        scene_provider=lambda _: search.CapturedScene(scene(), None),
        sampler=fake_sampler, task_factory=factory,
    ) is None
    assert len(calls) == 8
    assert calls[:4] == calls[4:]


def test_terminal_continuation_rejection_selects_another_preparation_pair():
    calls = []

    def validator(planned):
        calls.append(planned.preparation_joint_goals)
        return len(calls) == 3

    result = search.plan_preparation_candidates(
        None, TASK_PLAN, task_specs(), search_args(planning_attempts=2), LOGGER,
        scene_provider=lambda _: search.CapturedScene(scene(), None),
        sampler=fake_sampler, task_factory=lambda *a, **kw: (Task(True), None),
        solution_validator=validator,
    )
    assert result is not None and len(calls) == 3
    assert calls[0] == calls[1] and calls[1] != calls[2]
    assert result.preparation_joint_goals == calls[2]


def test_pipeline_failure_retries_same_candidate_until_complete_success():
    specs, calls = task_specs(), []
    specs[2].planner = "PipelinePlanner"

    class PipelineTask(Task):
        def __getitem__(self, name):
            failures = [NS(comment="INVALID_MOTION_PLAN")] if name == "follow_path" else []
            return NS(failures=failures)

    def factory(*_args, **kwargs):
        calls.append(copy.deepcopy(kwargs["preparation_joint_goals"]))
        return PipelineTask(len(calls) == 3), None

    result = search.plan_preparation_candidates(
        None, TASK_PLAN, specs, search_args(planning_attempts=3), LOGGER,
        scene_provider=lambda _: search.CapturedScene(scene(), None),
        sampler=fake_sampler, task_factory=factory,
    )
    assert result is not None
    assert len(calls) == 3
    assert calls == [calls[0]] * 3


def test_pipeline_retry_budget_advances_to_next_candidate():
    specs, calls = task_specs(), []
    for spec in specs:
        spec.planner = "PipelinePlanner"

    def factory(*_args, **kwargs):
        calls.append(copy.deepcopy(kwargs["preparation_joint_goals"]))
        return Task(len(calls) == 3), None

    assert search.plan_preparation_candidates(
        None, TASK_PLAN, specs, search_args(planning_attempts=2), LOGGER,
        scene_provider=lambda _: search.CapturedScene(scene(), None),
        sampler=fake_sampler, task_factory=factory,
    ) is not None
    assert calls[0] == calls[1]
    assert calls[2] != calls[1]


def test_budget_stops_search_between_native_planner_calls():
    now, calls = [0.0], []

    def factory(*_args, **_kwargs):
        calls.append(True)
        now[0] = 31.0
        return Task(False), None

    assert search.plan_preparation_candidates(
        None, TASK_PLAN, task_specs(), search_args(), LOGGER,
        scene_provider=lambda _: search.CapturedScene(scene(), None),
        sampler=fake_sampler, task_factory=factory,
        clock=lambda: now[0],
    ) is None
    assert len(calls) == 1


@pytest.mark.parametrize("failure", ["ik", "collision", "snapshot"])
def test_unusable_candidates_do_not_start_full_planning(failure):
    snapshot = scene()
    if failure == "collision":
        snapshot.is_state_valid = lambda *_: False

    def capture(_node):
        if failure == "snapshot":
            raise RuntimeError("scene unavailable")
        return search.CapturedScene(snapshot, None)

    def factory(*_args, **_kwargs):
        pytest.fail("invalid candidates must not be planned")

    assert search.plan_preparation_candidates(
        None, TASK_PLAN, task_specs(), search_args(), LOGGER,
        scene_provider=capture,
        sampler=(lambda *_a, **_k: []) if failure == "ik" else fake_sampler,
        task_factory=factory,
    ) is None


def test_diagonal_pair_order_is_complete_and_alternates_arms_early():
    pairs = list(search.candidate_pair_indices(3, 2))
    assert pairs[:3] == [(0, 0), (0, 1), (1, 0)]
    assert set(pairs) == {(i, j) for i in range(3) for j in range(2)}
    assert len(pairs) == 6


@pytest.mark.parametrize("argument", [
    "preparation-ik-candidates", "preparation-ik-attempts", "preparation-ik-timeout",
    "preparation-min-joint-distance", "preparation-candidate-attempts",
    "preparation-search-timeout",
])
def test_search_parameters_reject_zero(argument):
    with pytest.raises(SystemExit):
        _parse_args([f"--{argument}", "0"])


def test_target_transform_handles_rotated_frame_and_inverted_tool():
    transform = np.array([[0., -1., 0., 2.], [1., 0., 0., 3.],
                          [0., 0., 1., 4.], [0., 0., 0., 1.]])
    snapshot = NS(knows_frame_transform=lambda _: True,
                  get_frame_transform=lambda _: transform)
    target = PoseStamped()
    target.header.frame_id = "fixture"
    target.pose.position.x = 0.5
    target.pose.orientation = rpy_to_quaternion(math.pi, 0.0, 0.0)
    result = search.pose_in_model(snapshot, target)
    expected = PoseStamped().pose
    expected.position.x, expected.position.y, expected.position.z = 2.0, 3.5, 4.0
    expected.orientation = rpy_to_quaternion(math.pi, 0.0, math.pi / 2)
    assert search._pose_matches(result, expected)


@pytest.fixture
def ik_scene(tmp_path):
    import rclcpp
    from moveit.core.planning_scene import PlanningScene
    from moveit.task_constructor import core

    links = '<link name="base"/>'
    joints = ''
    for index, axis in enumerate(["0 0 1", "0 1 0", "1 0 0", "0 1 0", "1 0 0", "0 1 0", "0 0 1"]):
        links += f'<link name="link{index}"/>'
        parent = "base" if index == 0 else f"link{index-1}"
        joints += f'''<joint name="joint{index}" type="revolute">
          <parent link="{parent}"/><child link="link{index}"/>
          <origin xyz="0.15 0 0"/><axis xyz="{axis}"/>
          <limit lower="-2.8" upper="2.8" effort="10" velocity="1"/>
        </joint>'''
    urdf = f'<robot name="ik_test">{links}{joints}</robot>'
    srdf = ('<robot name="ik_test"><group name="arm">'
            '<chain base_link="base" tip_link="link6"/></group></robot>')
    parameters = tmp_path / "robot.yaml"
    parameters.write_text(yaml.safe_dump({"/**": {"ros__parameters": {
        "robot_description": urdf,
        "robot_description_semantic": srdf,
        "robot_description_kinematics": {"arm": {
            "kinematics_solver": "lma_kinematics_plugin/LMAKinematicsPlugin",
            "kinematics_solver_timeout": 0.05,
        }},
    }}}))
    rclcpp.init()
    try:
        options = rclcpp.NodeOptions(
            automatically_declare_parameters_from_overrides=True,
            arguments=["--ros-args", "--params-file", str(parameters)],
        )
        node = rclcpp.Node("preparation_ik_test", options)
        owner = core.Task(introspection=False)
        owner.loadRobotModel(node)
        snapshot = PlanningScene(owner.getRobotModel())
        state = copy.copy(snapshot.current_state)
        state.joint_positions = dict(zip(
            (f"joint{i}" for i in range(7)), [0.1, -0.5, 0.4, 1.0, 0.7, -0.9, 0.1],
        ))
        state.update()
        snapshot.current_state = state
        yield snapshot
    finally:
        rclcpp.shutdown()


def test_native_ik_samples_distinct_joints_without_changing_snapshot(ik_scene, monkeypatch):
    initial = ik_scene.current_state.joint_positions
    target = PoseStamped()
    target.header.frame_id = "base"
    target.pose = ik_scene.current_state.get_pose("link6")
    spec = NS(actor="leader", group="arm", ik_frame="link6")
    rng_factory = np.random.default_rng
    monkeypatch.setattr(search.np.random, "default_rng", lambda: rng_factory(7))
    candidates = search.sample_preparation_ik(
        ik_scene, target, spec,
        search_args(preparation_min_joint_distance=0.15, preparation_ik_attempts=80),
        LOGGER, time.monotonic() + 15.0,
    )
    assert len(candidates) >= 2
    assert ik_scene.current_state.joint_positions == initial
    for index, candidate in enumerate(candidates):
        state = copy.copy(ik_scene.current_state)
        state.joint_positions = candidate
        state.update()
        assert search._pose_matches(state.get_pose("link6"), target.pose)
        for previous in candidates[:index]:
            difference = np.array(list(candidate.values())) - list(previous.values())
            assert np.linalg.norm(difference) >= 0.15


def test_native_builder_fixes_preparation_joints_and_caches_continuation(ik_scene, monkeypatch):
    from moveit.task_constructor import core
    from dual_fr3_trunking_mtc.mtc import task_builder
    from dual_fr3_trunking_mtc.mtc.cached_execution import cache_selected_stages
    from dual_fr3_trunking_mtc.mtc.planning import PlannedTask, create_task_from_args

    def planners(*_args):
        cartesian = core.CartesianPath()
        cartesian.step_size = 0.002
        cartesian.jump_threshold = 1.5
        cartesian.min_fraction = 1.0
        jointspace = core.JointInterpolationPlanner()
        return cartesian, jointspace, jointspace

    monkeypatch.setattr(task_builder, "create_motion_planners", planners)
    initial = ik_scene.current_state.joint_positions
    goal = dict(initial)
    goal["joint0"] += 0.1
    prep = approach("leader", 0)
    prep.group, prep.ik_frame, prep.planner = "arm", "link6", "JointInterpolationPlanner"
    line = NS(name="continuation", stage_index=1, executable=True, phase="formal",
              mtc_stage_type="MoveRelative", primitive="follow", group="arm", ik_frame="link6",
              frame_id="base", vector=(0.01, 0.0, 0.0))
    specs = (prep, line)
    goals = {prep.name: goal}
    task, _ = create_task_from_args(
        None, TASK_PLAN, specs, search_args(), preparation_joint_goals=goals, start_scene=ik_scene,
    )
    assert task.plan(1)
    selected = PlannedTask(task, task.solutions[0], specs, goals)
    cached = cache_selected_stages(selected)
    assert len(cached) == 2
    assert ik_scene.current_state.joint_positions == initial
    assert cached[0].solution.start.scene.current_state.joint_positions == initial
    assert cached[0].solution.end.scene.current_state.joint_positions == pytest.approx(goal)
    first = cached[1].solution.start.scene.current_state.get_pose("link6")
    last = cached[1].solution.end.scene.current_state.get_pose("link6")
    assert last.position.x - first.position.x == pytest.approx(0.01, abs=1e-4)
    assert last.position.y == pytest.approx(first.position.y, abs=1e-4)
    assert last.position.z == pytest.approx(first.position.z, abs=1e-4)
