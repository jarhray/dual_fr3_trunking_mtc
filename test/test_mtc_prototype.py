import logging
import math
from inspect import signature
from types import SimpleNamespace

import pytest
from shape_msgs.msg import SolidPrimitive

from dual_fr3_trunking_mtc.models import Keypoint, TaskPlan
from dual_fr3_trunking_mtc.mtc.task_builder import (
    ANCHOR_PATH_CONSTRAINT_MIN_Z,
    ANCHOR_PATH_CONSTRAINT_XY_SIZE,
    DEFAULT_ANCHOR_MAX_PATH_Z,
    OMPL_MOVE_TO_TIMEOUT,
    OMPL_NUM_PLANNING_ATTEMPTS,
    OMPL_PIPELINE_NAME,
    OMPL_PLANNER_ID,
    create_motion_planners,
    max_link_z_path_constraint,
    planner_for_move_to,
)
from dual_fr3_trunking_mtc.mtc_prototype import _execute_stage_by_stage
from dual_fr3_trunking_mtc.planner import build_segment_plans
from dual_fr3_trunking_mtc.scheduler import build_task_plan
from dual_fr3_trunking_mtc.segment_executor import (
    StageRunner,
    build_segment_stages,
    stage_description,
)
from dual_fr3_trunking_mtc.stages.compiler import build_mtc_stage_specs
from dual_fr3_trunking_mtc.stages.specs import (
    mtc_stage_sequence_to_dict,
    mtc_stage_sequence_to_text,
)


_create_motion_planners = create_motion_planners
_max_link_z_path_constraint = max_link_z_path_constraint
_planner_for_move_to = planner_for_move_to

TASK_FRAME = "left_fr3_link0"


def _keypoint(name: str, x: float, y: float, in_slot: bool) -> Keypoint:
    return Keypoint(name, TASK_FRAME, (x, y, 0.0), in_slot)


def _task_plan(
    keypoints,
    initial_leader_index: int = 1,
    initial_follower_index: int = 0,
) -> TaskPlan:
    return build_task_plan(
        build_segment_plans(keypoints),
        initial_leader_index=initial_leader_index,
        initial_follower_index=initial_follower_index,
    )


def _assert_same_yaw(actual: float, expected: float) -> None:
    error = math.atan2(
        math.sin(actual - expected),
        math.cos(actual - expected),
    )
    assert error == pytest.approx(0.0)


def test_build_mtc_stage_specs_from_task_schedule():
    keypoints = [
        _keypoint("kp0", 0.0, 0.0, False),
        _keypoint("kp1", 0.0, 0.1, True),
        _keypoint("kp2", 0.0, 0.2, True),
        _keypoint("kp3", 0.0, 0.3, False),
    ]
    task_plan = _task_plan(
        keypoints,
        initial_leader_index=1,
        initial_follower_index=0,
    )

    specs = build_mtc_stage_specs(task_plan)

    assert [spec.primitive for spec in specs] == [
        "seat_cable_on_edge",
        "turn_gripper_to_next_keypoint",
        "leader_move_ahead_for_seat_edge",
        "turn_gripper_to_next_keypoint",
        "direct_move_to_seat_edge_keypoint",
        "direct_move_to_next_anchor",
        "turn_gripper_to_next_keypoint",
        "cartesian_move_to_next_keypoint",
        "seat_cable_on_edge",
    ]
    assert specs[0].executable is False
    assert specs[2].vector == pytest.approx((0.0, 0.10, 0.0))
    assert specs[5].primitive == "direct_move_to_next_anchor"
    assert specs[5].planner == "PipelinePlanner"
    assert specs[6].planner == "CartesianPath"
    assert specs[8].executable is False


def test_stage_compiler_accepts_task_plan_as_its_only_plan_input():
    parameters = signature(build_mtc_stage_specs).parameters

    assert next(iter(parameters)) == "task_plan"
    assert {
        "keypoints",
        "task_steps",
        "initial_leader_index",
        "initial_follower_index",
    }.isdisjoint(parameters)


def test_create_motion_planners_configures_move_group_ompl():
    class Planner:
        pass

    class PipelinePlanner(Planner):
        def __init__(self, node, pipeline):
            self.node = node
            self.pipeline = pipeline

    core = SimpleNamespace(
        CartesianPath=Planner,
        JointInterpolationPlanner=Planner,
        PipelinePlanner=PipelinePlanner,
    )
    node = object()

    cartesian, jointspace, ompl = _create_motion_planners(
        core,
        node,
        cartesian_step_size=0.01,
        motion_velocity_scaling=0.1,
        motion_acceleration_scaling=0.2,
    )

    assert cartesian.step_size == pytest.approx(0.01)
    assert cartesian.jump_threshold == pytest.approx(2.0)
    assert cartesian.min_fraction == pytest.approx(1.0)
    assert jointspace.max_velocity_scaling_factor == pytest.approx(0.1)
    assert ompl.node is node
    assert ompl.pipeline == OMPL_PIPELINE_NAME
    assert ompl.planner == OMPL_PLANNER_ID
    assert ompl.num_planning_attempts == OMPL_NUM_PLANNING_ATTEMPTS
    assert ompl.max_velocity_scaling_factor == pytest.approx(0.1)
    assert ompl.max_acceleration_scaling_factor == pytest.approx(0.2)
    assert OMPL_MOVE_TO_TIMEOUT == pytest.approx(5.0)


def test_move_to_planner_selection_uses_stage_metadata():
    cartesian = object()
    jointspace = object()
    ompl = object()

    assert (
        _planner_for_move_to(
            SimpleNamespace(planner="PipelinePlanner", name="anchor"),
            cartesian,
            jointspace,
            ompl,
        )
        is ompl
    )


def test_anchor_max_z_constraint_limits_tcp_in_keypoint_frame():
    constraints = _max_link_z_path_constraint(
        "left_fr3_link0",
        "left_fr3_hand_tcp",
        DEFAULT_ANCHOR_MAX_PATH_Z,
    )

    assert len(constraints.position_constraints) == 1
    position = constraints.position_constraints[0]
    assert position.header.frame_id == "left_fr3_link0"
    assert position.link_name == "left_fr3_hand_tcp"
    assert position.weight == pytest.approx(1.0)

    region = position.constraint_region.primitives[0]
    pose = position.constraint_region.primitive_poses[0]
    height = region.dimensions[SolidPrimitive.BOX_Z]
    assert region.type == SolidPrimitive.BOX
    assert region.dimensions[SolidPrimitive.BOX_X] == pytest.approx(
        ANCHOR_PATH_CONSTRAINT_XY_SIZE
    )
    assert pose.position.z + 0.5 * height == pytest.approx(
        DEFAULT_ANCHOR_MAX_PATH_Z
    )
    assert pose.position.z - 0.5 * height == pytest.approx(
        ANCHOR_PATH_CONSTRAINT_MIN_Z
    )


def test_anchor_max_z_constraint_rejects_invalid_upper_bound():
    with pytest.raises(ValueError, match="anchor_max_path_z"):
        _max_link_z_path_constraint("frame", "tcp", float("inf"))


@pytest.mark.parametrize("selected", [None, {0}, {1}, {2}])
def test_builder_applies_both_anchor_limits_and_absolute_recovery_goals(monkeypatch, selected):
    from dual_fr3_trunking_mtc.mtc import task_builder

    class Planner:
        def __init__(self, *_args):
            pass

    class Task:
        def __init__(self):
            self.stages = []

        def loadRobotModel(self, _node):
            pass

        def add(self, stage):
            self.stages.append(stage)

    class MoveTo:
        def __init__(self, name, planner):
            self.name = name

        def setGoal(self, goal):
            self.goal = goal

        def setCostTerm(self, cost):
            self.cost = cost

    core = SimpleNamespace(
        CartesianPath=Planner, JointInterpolationPlanner=Planner,
        PipelinePlanner=Planner, Task=Task,
    )
    stages = SimpleNamespace(CurrentState=lambda name: name, MoveTo=MoveTo)
    monkeypatch.setattr(task_builder, "import_mtc_modules", lambda: (None, core, stages))
    keypoints = [_keypoint("a", 0.0, 0.0, True), _keypoint("b", 0.0, 0.2, True)]
    specs = [
        SimpleNamespace(
            name=primitive, primitive=primitive, stage_index=index,
            executable=True, mtc_stage_type=kind, planner=planner,
            frame_id=TASK_FRAME, ik_frame="left_fr3_hand_tcp", group="left_fr3_arm",
            to_index=1, target_yaw=0.0,
        )
        for index, (primitive, planner, kind) in enumerate([
            ("direct_move_to_next_anchor", "PipelinePlanner", "MoveTo"),
            ("direct_move_to_seat_edge_keypoint", "JointInterpolationPlanner", "MoveTo"),
            ("cartesian_move_to_next_keypoint", "CartesianPath", "MoveRelative"),
        ])
    ]
    from geometry_msgs.msg import PoseStamped
    recovery_target = PoseStamped()
    recovery_target.header.frame_id = "world"
    recovery_target.pose.position.y = 0.2
    recovery_target.pose.orientation.w = 1.0
    task, _ = task_builder.create_mtc_task(
        object(), _task_plan(keypoints), specs, selected_stage_indices=selected,
        recovery_pose_goals={"cartesian_move_to_next_keypoint": recovery_target},
    )
    moves = task.stages[1:]
    assert len(moves) == (3 if selected is None else 1)
    for move in moves:
        is_anchor = move.name == "direct_move_to_next_anchor"
        assert hasattr(move, "cost") is (
            is_anchor or move.name == "cartesian_move_to_next_keypoint"
        )
        assert hasattr(move, "path_constraints") is is_anchor
        if move.name == "cartesian_move_to_next_keypoint":
            # Recovery uses the saved endpoint, not the original relative vector.
            assert move.goal is recovery_target


def test_mtc_stage_sequence_interface_includes_execution_order():
    keypoints = [
        _keypoint("kp0", 0.0, 0.0, False),
        _keypoint("kp1", 0.0, 0.1, True),
    ]
    task_plan = _task_plan(
        keypoints,
        initial_leader_index=1,
        initial_follower_index=0,
    )
    specs = build_mtc_stage_specs(task_plan)

    sequence = mtc_stage_sequence_to_dict(specs)
    text = mtc_stage_sequence_to_text(specs)

    assert sequence["interface_version"] == 2
    assert sequence["stage_count"] == 1
    assert sequence["stage_sequence"][0]["mtc_stage_type"] == "InfoOnly"
    assert sequence["stage_sequence"][0]["primitive"] == "seat_cable_on_edge"
    assert sequence["stage_sequence"][0]["group"] == "right_fr3_arm"
    assert sequence["stage_sequence"][0]["actor"] == "follower"
    assert sequence["stage_sequence"][0]["execution_order"] == [
        "follower_seat_cable_on_edge",
    ]
    assert "stage_key=step_0:seat_edge:follower:kp0->kp1:seat_cable_on_edge" in text


def test_leader_lead_distance_must_be_positive():
    keypoints = [
        _keypoint("kp0", 0.0, 0.0, False),
        _keypoint("kp1", 0.1, 0.0, True),
    ]
    task_plan = _task_plan(keypoints)

    with pytest.raises(ValueError, match="leader_lead_distance"):
        build_mtc_stage_specs(
            task_plan,
            leader_lead_distance=0.0,
        )


def test_formal_stage_specs_exclude_preparation_actions():
    keypoints = [
        _keypoint("kp0", 0.0, 0.0, False),
        _keypoint("kp1", 0.0, 0.1, True),
    ]
    specs = build_mtc_stage_specs(_task_plan(keypoints))

    assert all(spec.action != "preparation" for spec in specs)
    assert [spec.primitive for spec in specs] == ["seat_cable_on_edge"]


def test_seat_edge_can_insert_a_profile_based_gripper_operation():
    keypoints = [
        _keypoint("kp0", 0.0, 0.0, False),
        Keypoint(
            "kp1",
            TASK_FRAME,
            (0.0, 0.1, 0.0),
            True,
            metadata={
                "gripper": {
                    "actor": "follower",
                    "profile": "trunk_edge",
                    "action": "grasp",
                    "width": 0.018,
                }
            },
        ),
    ]

    specs = build_mtc_stage_specs(_task_plan(keypoints))

    assert [spec.mtc_stage_type for spec in specs] == [
        "InfoOnly",
        "GripperOperation",
    ]
    gripper = specs[1]
    assert gripper.group == "right_fr3_hand"
    assert gripper.gripper_profile == "trunk_edge"
    assert gripper.gripper_action == "grasp"
    assert gripper.gripper_width_override == pytest.approx(0.018)


def test_default_orientation_directions_face_leader_and_follower_oppositely():
    keypoints = [
        _keypoint("kp0", 0.0, 0.0, False),
        _keypoint("kp1", 0.0, 0.1, True),
        _keypoint("kp2", 0.0, 0.2, True),
        _keypoint("kp3", 0.0, 0.3, False),
    ]
    specs = build_mtc_stage_specs(
        _task_plan(keypoints),
    )
    oriented_arm_specs = [
        spec
        for spec in specs
        if spec.group.endswith("_arm") and spec.mtc_stage_type != "InfoOnly"
    ]
    for spec in oriented_arm_specs:
        expected = math.pi / 2.0 if spec.actor == "follower" else -math.pi / 2.0
        _assert_same_yaw(spec.target_yaw, expected)


def test_forward_forward_orientation_directions_restore_legacy_yaw():
    keypoints = [
        _keypoint("kp0", 0.0, 0.0, False),
        _keypoint("kp1", 0.0, 0.1, True),
        _keypoint("kp2", 0.0, 0.2, True),
        _keypoint("kp3", 0.0, 0.3, False),
    ]
    specs = build_mtc_stage_specs(
        _task_plan(keypoints),
        leader_orientation_direction="forward",
        follower_orientation_direction="forward",
    )
    leader_lead = next(
        spec
        for spec in specs
        if spec.primitive == "leader_move_ahead_for_seat_edge"
    )

    _assert_same_yaw(leader_lead.target_yaw, math.pi / 2.0)


@pytest.mark.parametrize(
    "direction_arguments",
    [
        {"leader_orientation_direction": "sideways"},
        {"follower_orientation_direction": "sideways"},
    ],
)
def test_build_mtc_stage_specs_rejects_invalid_orientation_direction(
    direction_arguments,
):
    keypoints = [
        _keypoint("kp0", 0.0, 0.0, False),
        _keypoint("kp1", 0.1, 0.0, True),
    ]

    with pytest.raises(ValueError, match="orientation direction"):
        build_mtc_stage_specs(
            _task_plan(keypoints),
            **direction_arguments,
        )


def test_leader_lead_translation_stays_forward_while_target_yaw_is_reversed():
    keypoints = [
        _keypoint("kp0", 0.0, 0.0, False),
        _keypoint("kp1", 0.0, 0.1, True),
        _keypoint("kp2", 0.0, 0.2, True),
        _keypoint("kp3", 0.0, 0.3, False),
    ]
    specs = build_segment_stages(keypoints)
    leader_lead = next(
        spec
        for spec in specs
        if spec.primitive == "leader_move_ahead_for_seat_edge"
    )

    assert leader_lead.vector == pytest.approx((0.0, 0.10, 0.0))
    _assert_same_yaw(leader_lead.target_yaw, -math.pi / 2.0)


def test_segment_executor_relative_target_uses_stage_target_yaw():
    keypoints = [
        _keypoint("kp0", 0.0, 0.0, False),
        _keypoint("kp1", 0.0, 0.1, True),
        _keypoint("kp2", 0.0, 0.2, True),
        _keypoint("kp3", 0.0, 0.3, False),
    ]
    leader_lead = next(
        spec
        for spec in build_segment_stages(keypoints)
        if spec.primitive == "leader_move_ahead_for_seat_edge"
    )
    runner = object.__new__(StageRunner)
    runner.keypoints = keypoints
    current_pose = SimpleNamespace(
        position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
    )

    position, yaw = runner._target(leader_lead, current_pose)

    assert position == pytest.approx((1.0, 2.1, 3.0))
    _assert_same_yaw(yaw, leader_lead.target_yaw)
    _assert_same_yaw(yaw, -math.pi / 2.0)


def test_forward_forward_turn_specs_use_legacy_actor_current_yaw():
    keypoints = [
        _keypoint("entry_0", 0.6, 0.70, False),
        _keypoint("corner_1", 0.6, 0.45, True),
        _keypoint("corner_2", 0.6, 0.35, True),
        _keypoint("corner_3", 0.4, 0.35, True),
        _keypoint("corner_4", 0.4, 0.25, True),
        _keypoint("entry_5", 0.4, 0.15, False),
    ]
    specs = build_mtc_stage_specs(
        _task_plan(keypoints),
        leader_orientation_direction="forward",
        follower_orientation_direction="forward",
    )
    turns = [
        spec for spec in specs
        if spec.primitive == "turn_gripper_to_next_keypoint"
    ]

    assert [turn.yaw_delta for turn in turns] == pytest.approx(
        [
            0.0,
            0.0,
            0.0,
            -math.pi / 2.0,
            math.pi / 2.0,
        ]
    )
    assert [turn.executable for turn in turns] == [
        False,
        False,
        False,
        True,
        True,
    ]


def test_segment_executor_uses_the_mtc_stage_order():
    keypoints = [
        _keypoint("entry_0", 0.6, 0.70, False),
        _keypoint("corner_1", 0.6, 0.45, True),
        _keypoint("corner_2", 0.6, 0.35, True),
        _keypoint("corner_3", 0.4, 0.35, True),
        _keypoint("corner_4", 0.4, 0.25, True),
        _keypoint("entry_5", 0.4, 0.15, False),
    ]

    specs = build_segment_stages(keypoints)

    assert len(specs) == 13
    assert specs[0].actor == "follower"
    assert specs[0].to_keypoint == "corner_1"
    assert specs[2].primitive == "leader_move_ahead_for_seat_edge"
    assert specs[5].primitive == "direct_move_to_next_anchor"
    assert specs[-1].primitive == "seat_cable_on_edge"
    assert "cartesian" in stage_description(specs[9])


def _stage_execution_args():
    return SimpleNamespace(
        leader_group="left_fr3_arm",
        follower_group="right_fr3_arm",
        leader_ik_frame="left_fr3_hand_tcp",
        follower_ik_frame="right_fr3_hand_tcp",
        cartesian_step_size=0.01,
        motion_velocity_scaling=0.1,
        motion_acceleration_scaling=0.1,
        anchor_max_path_z=DEFAULT_ANCHOR_MAX_PATH_Z,
        anchor_max_path_length_ratio=1.5,
        initial_leader_index=1,
        initial_follower_index=0,
        leader_lead_distance=0.1,
        leader_orientation_direction="reverse",
        follower_orientation_direction="forward",
        tool_roll=math.pi,
        tool_pitch=0.0,
    )


def test_stage_execution_halts_after_first_execution_failure(monkeypatch):
    created_stage_indices = []

    class FakeTask:
        solutions = [object()]

        @staticmethod
        def plan():
            return True

        @staticmethod
        def execute(_solution):
            return False

    def fake_create_mtc_task(*_args, selected_stage_indices, **_kwargs):
        assert _kwargs["anchor_max_path_length_ratio"] == 1.5
        created_stage_indices.append(next(iter(selected_stage_indices)))
        return FakeTask(), []

    monkeypatch.setattr(
        "dual_fr3_trunking_mtc.mtc_prototype.create_mtc_task",
        fake_create_mtc_task,
    )
    specs = [
        SimpleNamespace(executable=True, stage_index=0, name="first"),
        SimpleNamespace(executable=True, stage_index=1, name="must_not_run"),
    ]

    succeeded = _execute_stage_by_stage(
        object(),
        specs,
        TaskPlan((), (), (), 1, 0),
        _stage_execution_args(),
        logging.getLogger("test_stage_halt"),
    )

    assert succeeded is False
    assert created_stage_indices == [0]


def test_stage_execution_halts_after_planning_exception(monkeypatch):
    created_stage_indices = []

    def fake_create_mtc_task(*_args, selected_stage_indices, **_kwargs):
        created_stage_indices.append(next(iter(selected_stage_indices)))
        raise RuntimeError("planner failure")

    monkeypatch.setattr(
        "dual_fr3_trunking_mtc.mtc_prototype.create_mtc_task",
        fake_create_mtc_task,
    )
    specs = [
        SimpleNamespace(executable=True, stage_index=0, name="first"),
        SimpleNamespace(executable=True, stage_index=1, name="must_not_run"),
    ]

    succeeded = _execute_stage_by_stage(
        object(),
        specs,
        TaskPlan((), (), (), 1, 0),
        _stage_execution_args(),
        logging.getLogger("test_stage_exception_halt"),
    )

    assert succeeded is False
    assert created_stage_indices == [0]


def test_stage_execution_dispatches_gripper_without_arm_planning(monkeypatch):
    arm_tasks_created = []
    requests = []

    def fake_create_mtc_task(*_args, **_kwargs):
        arm_tasks_created.append(True)
        raise AssertionError("arm planning must not run for a gripper stage")

    class FakeGripperController:
        @staticmethod
        def execute(request):
            requests.append(request)
            return True

    monkeypatch.setattr(
        "dual_fr3_trunking_mtc.mtc_prototype.create_mtc_task",
        fake_create_mtc_task,
    )
    spec = SimpleNamespace(
        executable=True,
        stage_index=0,
        name="seat_edge_grasp",
        mtc_stage_type="GripperOperation",
        actor="follower",
        gripper_profile="cable_body",
        gripper_action="",
        gripper_width_override=None,
    )

    succeeded = _execute_stage_by_stage(
        object(),
        [spec],
        TaskPlan((), (), (), 1, 0),
        _stage_execution_args(),
        logging.getLogger("test_gripper_stage"),
        gripper_controller=FakeGripperController(),
    )

    assert succeeded is True
    assert not arm_tasks_created
    assert len(requests) == 1
    assert requests[0].actor == "follower"
    assert requests[0].profile == "cable_body"


def test_confirmation_required_stage_fails_closed_without_callback(monkeypatch):
    controller_calls = []

    class FakeGripperController:
        @staticmethod
        def execute(_request):
            controller_calls.append(True)
            return True

    spec = SimpleNamespace(
        executable=True,
        confirmation_required=True,
        stage_index=0,
        stage_key="preparation:leader:close",
        name="leader_close",
        mtc_stage_type="GripperOperation",
        actor="leader",
        gripper_profile="cable_tip",
        gripper_action="",
        gripper_width_override=None,
    )

    succeeded = _execute_stage_by_stage(
        object(),
        [spec],
        TaskPlan((), (), (), 1, 0),
        _stage_execution_args(),
        logging.getLogger("test_confirmation_fail_closed"),
        gripper_controller=FakeGripperController(),
    )

    assert succeeded is False
    assert not controller_calls
