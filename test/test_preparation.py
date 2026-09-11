import logging
import math
from types import SimpleNamespace

import pytest

import dual_fr3_trunking_mtc.preparation as preparation
import dual_fr3_trunking_mtc.mtc.task_builder as task_builder
from dual_fr3_trunking_mtc.models import Keypoint
from dual_fr3_trunking_mtc.mtc.task_builder import create_mtc_task
from dual_fr3_trunking_mtc.planner import build_segment_plans
from dual_fr3_trunking_mtc.preparation import (
    DUAL_DESCENT,
    FOLLOWER_APPROACH,
    FOLLOWER_CLOSE,
    LEADER_APPROACH,
    LEADER_CLOSE,
    PreparationConfig,
    build_preparation_stage_specs,
    confirm_stage,
)
from dual_fr3_trunking_mtc.runtime.config import DEFAULTS
from dual_fr3_trunking_mtc.scheduler import build_task_plan
from dual_fr3_trunking_mtc.stages.compiler import build_mtc_stage_specs


TASK_FRAME = "left_fr3_link0"


def _keypoint(name, x, y, z=0.05):
    return Keypoint(name, TASK_FRAME, (x, y, z), True)


def _task_plan():
    keypoints = [
        _keypoint("kp0", 0.6, 0.7),
        _keypoint("kp1", 0.6, 0.4),
        _keypoint("kp2", 0.6, 0.1),
    ]
    return build_task_plan(build_segment_plans(keypoints))


def _assert_same_yaw(actual, expected):
    error = math.atan2(math.sin(actual - expected), math.cos(actual - expected))
    assert error == pytest.approx(0.0)


def test_preparation_uses_the_same_stage_spec_sequence_as_formal_actions():
    specs = build_preparation_stage_specs(_task_plan(), PreparationConfig())

    assert [spec.execution_order[0] for spec in specs] == [
        LEADER_APPROACH,
        FOLLOWER_APPROACH,
        LEADER_CLOSE,
        FOLLOWER_CLOSE,
        DUAL_DESCENT,
    ]
    assert all(spec.phase == "preparation" for spec in specs)
    assert [spec.confirmation_required for spec in specs] == [
        False,
        False,
        True,
        True,
        True,
    ]


def test_stage_compiler_returns_one_indexed_preparation_and_formal_program():
    specs = build_mtc_stage_specs(
        _task_plan(),
        preparation_config=PreparationConfig(),
    )

    assert [spec.stage_index for spec in specs] == list(range(len(specs)))
    assert [spec.phase for spec in specs[:5]] == ["preparation"] * 5
    assert specs[5].phase == "formal"


def test_preparation_approach_uses_task_plan_indices_height_and_direction():
    specs = build_preparation_stage_specs(_task_plan(), PreparationConfig())
    leader, follower = specs[:2]

    assert leader.from_index == 1
    assert follower.from_index == 0
    assert leader.vector == pytest.approx((0.0, 0.0, DEFAULTS.preparation_height))
    assert follower.vector == pytest.approx(
        (0.0, 0.0, DEFAULTS.preparation_height)
    )
    _assert_same_yaw(leader.target_yaw, math.pi / 2.0)
    _assert_same_yaw(follower.target_yaw, -math.pi / 2.0)


def test_preparation_descent_is_a_merger_with_two_stage_spec_children():
    descent = build_preparation_stage_specs(
        _task_plan(),
        PreparationConfig(),
    )[-1]

    assert descent.mtc_stage_type == "Merger"
    assert [child.actor for child in descent.children] == ["leader", "follower"]
    assert [child.group for child in descent.children] == [
        "left_fr3_arm",
        "right_fr3_arm",
    ]
    assert [child.vector[2] for child in descent.children] == pytest.approx(
        [-DEFAULTS.preparation_height, -DEFAULTS.preparation_height]
    )


def test_shared_task_builder_materializes_preparation_specs(monkeypatch):
    class Planner:
        pass

    class PipelinePlanner(Planner):
        def __init__(self, node, pipeline):
            self.node = node
            self.pipeline = pipeline

    class Stage:
        def __init__(self, name, planner=None):
            self.name = name
            self.planner = planner
            self.goal = None
            self.direction = None

        def setGoal(self, goal):
            self.goal = goal

        def setDirection(self, direction):
            self.direction = direction

        def setCostTerm(self, cost):
            self.cost = cost

    class Merger(Stage):
        def __init__(self, name):
            super().__init__(name)
            self.children = []

        def insert(self, stage):
            self.children.append(stage)

    class Task:
        def __init__(self):
            self.stages = []
            self.solutions = []

        def loadRobotModel(self, _node):
            pass

        def add(self, stage):
            self.stages.append(stage)

    core = SimpleNamespace(
        CartesianPath=Planner,
        JointInterpolationPlanner=Planner,
        PipelinePlanner=PipelinePlanner,
        Merger=Merger,
        Task=Task,
    )
    stages = SimpleNamespace(
        CurrentState=Stage,
        MoveTo=Stage,
        MoveRelative=Stage,
    )
    monkeypatch.setattr(
        task_builder,
        "import_mtc_modules",
        lambda: (object(), core, stages),
    )
    task_plan = _task_plan()
    preparation_specs = build_preparation_stage_specs(
        task_plan,
        PreparationConfig(),
    )

    task, materialized = create_mtc_task(
        object(),
        task_plan,
        [preparation_specs[0], preparation_specs[-1]],
    )

    assert materialized == [preparation_specs[0], preparation_specs[-1]]
    assert task.stages[1].group == "left_fr3_arm"
    assert task.stages[1].goal.pose.position.z == pytest.approx(
        0.05 + DEFAULTS.preparation_height
    )
    merger = task.stages[2]
    assert [child.group for child in merger.children] == [
        "left_fr3_arm",
        "right_fr3_arm",
    ]


def test_noninteractive_preparation_specs_do_not_request_confirmation():
    specs = build_preparation_stage_specs(
        _task_plan(),
        PreparationConfig(interactive=False),
    )

    assert not any(spec.confirmation_required for spec in specs)


def test_preparation_confirmation_uses_stage_description(caplog):
    spec = build_preparation_stage_specs(_task_plan(), PreparationConfig())[2]
    prompts = []

    assert confirm_stage(
        spec,
        lambda prompt: prompts.append(prompt) or "",
        logging.getLogger("test_preparation"),
    )
    assert prompts == [""]
    assert "WAITING FOR KEYBOARD CONFIRMATION" in caplog.text


def test_controlling_terminal_reader_uses_tty_directly(monkeypatch):
    class FakeTerminal:
        def __enter__(self):
            return self

        @staticmethod
        def __exit__(_exc_type, _exc_value, _traceback):
            return False

        @staticmethod
        def readline():
            return "\n"

    opened = []

    def fake_open(path, mode, **kwargs):
        opened.append((path, mode, kwargs))
        return FakeTerminal()

    monkeypatch.setattr("builtins.open", fake_open)

    assert preparation.read_controlling_terminal("") == "\n"
    assert opened == [
        (
            preparation.TERMINAL_DEVICE,
            "r",
            {"encoding": "utf-8", "buffering": 1},
        )
    ]
