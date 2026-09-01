import logging
import math
from types import SimpleNamespace

import pytest

import dual_fr3_trunking_mtc.preparation as preparation
from dual_fr3_trunking_mtc.models import Keypoint
from dual_fr3_trunking_mtc.preparation import (
    DUAL_DESCENT,
    FOLLOWER_APPROACH,
    FOLLOWER_CLOSE,
    LEADER_APPROACH,
    LEADER_CLOSE,
    PreparationConfig,
    _approach_pose,
    add_preparation_stages,
    build_preparation_steps,
    run_preparation,
)


TASK_FRAME = "left_fr3_link0"


def _keypoint(name, x, y, z=0.05):
    return Keypoint(name, TASK_FRAME, (x, y, z), True)


def _yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0
        * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2),
    )


def _assert_same_yaw(actual, expected):
    error = math.atan2(math.sin(actual - expected), math.cos(actual - expected))
    assert error == pytest.approx(0.0)


def test_preparation_sequence_matches_requested_order():
    keypoints = [
        _keypoint("kp0", 0.6, 0.7),
        _keypoint("kp1", 0.6, 0.4),
        _keypoint("kp2", 0.6, 0.1),
    ]

    steps = build_preparation_steps(keypoints, PreparationConfig())

    assert [step.key for step in steps] == [
        LEADER_APPROACH,
        FOLLOWER_APPROACH,
        LEADER_CLOSE,
        FOLLOWER_CLOSE,
        DUAL_DESCENT,
    ]
    assert [step.confirmation_required for step in steps] == [
        False,
        False,
        True,
        True,
        True,
    ]


def test_preparation_approach_uses_height_and_actor_orientation_direction():
    keypoints = [
        _keypoint("kp0", 0.6, 0.7),
        _keypoint("kp1", 0.6, 0.4),
        _keypoint("kp2", 0.6, 0.1),
    ]
    config = PreparationConfig()

    leader = _approach_pose(
        keypoints,
        config.leader_index,
        config.approach_height,
        config.tool_roll,
        config.tool_pitch,
        config.leader_orientation_direction,
    )
    follower = _approach_pose(
        keypoints,
        config.follower_index,
        config.approach_height,
        config.tool_roll,
        config.tool_pitch,
        config.follower_orientation_direction,
    )

    assert leader.pose.position.z == pytest.approx(0.20)
    assert follower.pose.position.z == pytest.approx(0.20)
    _assert_same_yaw(_yaw_from_quaternion(leader.pose.orientation), math.pi / 2.0)
    _assert_same_yaw(
        _yaw_from_quaternion(follower.pose.orientation),
        -math.pi / 2.0,
    )


def test_reverse_orientation_at_last_preparation_keypoint_uses_incoming_path():
    keypoints = [
        _keypoint("kp0", 0.0, 0.0),
        _keypoint("kp1", 1.0, 0.0),
    ]

    pose = _approach_pose(
        keypoints,
        index=1,
        height=0.15,
        tool_roll=math.pi,
        tool_pitch=0.0,
        orientation_direction="reverse",
    )

    _assert_same_yaw(_yaw_from_quaternion(pose.pose.orientation), math.pi)


def test_preparation_builds_a_merger_for_synchronized_cartesian_descent():
    class Planner:
        pass

    class PipelinePlanner(Planner):
        def __init__(self, node, pipeline):
            self.node = node
            self.pipeline = pipeline

    class Move:
        def __init__(self, name, planner):
            self.name = name
            self.planner = planner
            self.goal = None
            self.direction = None

        def setGoal(self, goal):
            self.goal = goal

        def setDirection(self, direction):
            self.direction = direction

    class Merger:
        def __init__(self, name):
            self.name = name
            self.children = []

        def insert(self, stage):
            self.children.append(stage)

    class Task:
        def __init__(self):
            self.stages = []

        def add(self, stage):
            self.stages.append(stage)

    core = SimpleNamespace(
        CartesianPath=Planner,
        JointInterpolationPlanner=Planner,
        PipelinePlanner=PipelinePlanner,
        Merger=Merger,
    )
    stages = SimpleNamespace(MoveTo=Move, MoveRelative=Move)
    task = Task()
    keypoints = [
        _keypoint("kp0", 0.6, 0.7),
        _keypoint("kp1", 0.6, 0.4),
        _keypoint("kp2", 0.6, 0.1),
    ]

    add_preparation_stages(
        task,
        core,
        stages,
        object(),
        keypoints,
        PreparationConfig(),
    )

    assert len(task.stages) == 5
    assert task.stages[0].group == "left_fr3_arm"
    assert task.stages[0].goal.pose.position.z == pytest.approx(0.20)
    assert task.stages[1].group == "right_fr3_arm"
    assert task.stages[2].group == "left_fr3_hand"
    assert task.stages[2].goal == {"left_fr3_finger_joint1": 0.0}
    assert task.stages[3].group == "right_fr3_hand"

    merger = task.stages[4]
    assert merger.name == "preparation_dual_cartesian_descent"
    assert [child.group for child in merger.children] == [
        "left_fr3_arm",
        "right_fr3_arm",
    ]
    assert [child.direction.vector.z for child in merger.children] == pytest.approx(
        [-0.15, -0.15]
    )


def test_interactive_preparation_prompts_only_for_gripping_and_descent(
    monkeypatch,
    caplog,
):
    created_steps = []
    prompts = []

    class FakeTask:
        solutions = [object()]

        @staticmethod
        def plan():
            return True

        @staticmethod
        def execute(_solution):
            return True

    def fake_create_task(
        _node,
        _core,
        _stages,
        _keypoints,
        _config,
        selected_step_keys,
    ):
        created_steps.append(next(iter(selected_step_keys)))
        return FakeTask(), []

    monkeypatch.setattr(preparation, "create_preparation_task", fake_create_task)
    keypoints = [
        _keypoint("kp0", 0.6, 0.7),
        _keypoint("kp1", 0.6, 0.4),
        _keypoint("kp2", 0.6, 0.1),
    ]

    succeeded = run_preparation(
        object(),
        object(),
        object(),
        keypoints,
        PreparationConfig(interactive=True),
        logging.getLogger("test_preparation"),
        input_fn=lambda prompt: prompts.append(prompt) or "",
    )

    assert succeeded is True
    assert created_steps == [
        LEADER_APPROACH,
        FOLLOWER_APPROACH,
        LEADER_CLOSE,
        FOLLOWER_CLOSE,
        DUAL_DESCENT,
    ]
    assert len(prompts) == 3
    assert caplog.text.count("WAITING FOR KEYBOARD CONFIRMATION") == 3


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

    assert preparation._read_controlling_terminal("") == "\n"
    assert opened == [
        (
            preparation.TERMINAL_DEVICE,
            "r",
            {"encoding": "utf-8", "buffering": 1},
        )
    ]
