"""Verify failure attribution and native collision/height diagnostics."""

import logging
from types import SimpleNamespace as NS

import pytest

from dual_fr3_trunking_mtc.mtc.diagnostics import (
    PlanningFailureHistory,
    _invalid_trajectory_details,
    log_planning_failure,
)


LOGGER = logging.getLogger(__name__)


def spec(name, index=0, **kwargs):
    """Build an anchor specification with optional overrides."""
    values = dict(
        name=name, stage_index=index, executable=True, phase="formal", actor="leader",
        group="left_fr3_arm", planner="PipelinePlanner", from_keypoint="p2",
        to_keypoint="entry_5", mtc_stage_type="MoveTo", primitive="direct_move_to_next_anchor",
        from_index=0, to_index=1, frame_id="base", ik_frame="tcp", target_yaw=1.2,
        children=(),
    )
    return NS(**(values | kwargs))


def stage(successes=0, *reasons):
    """Represent native stage result collections."""
    return NS(solutions=[object()] * successes, failures=[NS(comment=text) for text in reasons])


PLAN = NS(keypoints=[NS(position=(0.6, 0.373, 0.05), frame_id="base"),
                     NS(position=(0.364, -0.07, 0.1), frame_id="base")])


def test_report_distinguishes_failed_unreached_and_successful_candidates(caplog):
    """A rejected branch must not mask a valid branch or implicate later stages."""
    caplog.set_level(logging.INFO)
    task = {
        "approach": stage(1, "a rejected IK branch"),
        "anchor": stage(0, "PATH_LENGTH_LIMIT: TCP path 1.3 m; limit 0.76 m",
                        "INVALID_MOTION_PLAN"),
        "follow": stage(),
    }
    history = PlanningFailureHistory()
    log_planning_failure(
        task, [spec("approach"), spec("anchor", 1), spec("follow", 2)], LOGGER,
        task_plan=PLAN, history=history,
    )
    assert "approach HAS_SOLUTION (solutions=1, failures=1)" in caplog.text
    assert "anchor FAILED (solutions=0, failures=2)" in caplog.text
    assert "follow NO_RESULT" in caplog.text
    assert "actor=leader group=left_fr3_arm planner=PipelinePlanner p2 -> entry_5" in caplog.text
    assert "target=(0.3640, -0.0700, 0.1000) m frame=base" in caplog.text
    assert "TCP path 1.3 m; limit 0.76 m" in caplog.text
    assert "No stored trajectory/scene" in caplog.text
    assert history.reasons[("anchor", "PATH_LENGTH_LIMIT")] == 1
    assert history.reasons[("anchor", "INVALID_MOTION_PLAN")] == 1
    history.log_summary(LOGGER)
    assert "[planning-summary] 1 unsuccessful full-task attempts" in caplog.text


def test_nested_stage_failure_and_absent_hold_are_reported(caplog):
    """Attribute Merger child failures and distinguish absent hold operations."""
    caplog.set_level(logging.INFO)

    class Merger(dict):
        solutions = []
        failures = []

    child = spec("right_descent", 2, actor="follower", group="right_fr3_arm")
    merger = Merger(right_descent=stage(0, "Cartesian path incomplete"))
    log_planning_failure(
        {"descent": merger},
        [spec("descent", 1, mtc_stage_type="Merger", children=(child,)),
         spec("hold", 3, mtc_stage_type="GripperOperation")], LOGGER,
    )
    assert "right_descent FAILED" in caplog.text
    assert "Cartesian path incomplete" in caplog.text
    assert "hold SKIPPED" in caplog.text


def test_summary_uses_result_counts_not_total_attempts(caplog):
    """Earlier failures reduce later opportunities, not later success rates."""
    caplog.set_level(logging.INFO)
    specs = [spec(name, index) for index, name in enumerate(
        ("anchor", "follow", "turn", "later"),
    )]
    history = PlanningFailureHistory()
    attempts = [
        {"anchor": stage(0, "PATH_LENGTH_LIMIT: length 1.3 m", "INVALID_MOTION_PLAN"),
         "follow": stage(), "turn": stage(), "later": stage()},
        {"anchor": stage(1, "INVALID_MOTION_PLAN"),
         "follow": stage(0, "CartesianPath: incomplete", "CartesianPath: incomplete"),
         "turn": stage(), "later": stage()},
        {"anchor": stage(1), "follow": stage(1),
         "turn": stage(0, "CartesianPath: incomplete"), "later": stage()},
    ]
    for task in attempts:
        log_planning_failure(task, specs, LOGGER, history=history)
    caplog.clear()
    history.log_summary(LOGGER)
    lines = {
        name: next(line for line in caplog.text.splitlines() if f"] {name} |" in line)
        for name in ("anchor", "follow", "turn", "later")
    }
    assert "with_result=3/3 | has_solution=2 | failed_only=1 | no_result=0" in lines["anchor"]
    assert "failed_only/with_result=33.3%" in lines["anchor"]
    assert "with_result=2/3 | has_solution=1 | failed_only=1 | no_result=1" in lines["follow"]
    assert "failed_only/with_result=50.0%" in lines["follow"]
    assert "with_result=1/3 | has_solution=0 | failed_only=1 | no_result=2" in lines["turn"]
    assert "failed_only/with_result=100.0%" in lines["turn"]
    assert "with_result=0/3" in lines["later"]
    assert "failed_only/with_result=n/a" in lines["later"]
    assert history.reasons[("follow", "CartesianPath")] == 1
    assert history.reasons[("anchor", "INVALID_MOTION_PLAN")] == 2


def test_summary_excludes_skipped_missing_and_unselected_stages(caplog):
    """Recovery selection and absent native stages must not inflate denominators."""
    caplog.set_level(logging.INFO)
    history = PlanningFailureHistory()
    log_planning_failure(
        {"failed": stage(0, "TIMED_OUT")},
        [spec("completed", 0), spec("failed", 1),
         spec("hold", 2, mtc_stage_type="GripperOperation"),
         spec("missing", 3), spec("note", 4, executable=False)],
        LOGGER, selected_stage_indices={1, 2, 3, 4}, history=history,
    )
    assert (0, "completed") not in history.stage_outcomes
    assert history.stage_outcomes[(1, "failed")] == {"FAILED": 1}
    assert history.stage_outcomes[(2, "hold")] == {"SKIPPED": 1}
    assert history.stage_outcomes[(3, "missing")] == {"UNAVAILABLE": 1}
    assert history.stage_outcomes[(4, "note")] == {"SKIPPED": 1}
    caplog.clear()
    history.log_summary(LOGGER)
    assert "with_result=0/1" in caplog.text
    assert "skipped=1" in caplog.text
    assert "unavailable=1" in caplog.text


def test_recovery_reports_selected_stages_and_actual_absolute_target(caplog):
    """Recovery diagnostics describe only remaining motions and their saved goals."""
    caplog.set_level(logging.INFO)
    target = NS(header=NS(frame_id="world"), pose=NS(position=NS(x=0.1, y=0.2, z=0.3)))
    log_planning_failure(
        {"anchor": stage(0, "TIMED_OUT")}, [spec("completed"), spec("anchor", 1)], LOGGER,
        task_plan=PLAN, selected_stage_indices={1}, recovery_pose_goals={"anchor": target},
    )
    assert "completed" not in caplog.text
    assert "recovery_target=(0.1000, 0.2000, 0.3000) m frame=world" in caplog.text


def test_diagnostics_error_keeps_original_reason_and_does_not_raise(caplog):
    """An unsupported native diagnostic must not hide the original failure."""
    failure = NS(comment="INVALID_MOTION_PLAN", trajectory=object(), start=object())
    log_planning_failure(
        {"anchor": NS(solutions=[], failures=[failure])}, [spec("anchor")], LOGGER,
    )
    assert "INVALID_MOTION_PLAN" in caplog.text
    assert "Trajectory diagnostics unavailable" in caplog.text


def test_real_failed_trajectory_reports_collision_pair_and_height(tmp_path, capfd):
    """Use installed MoveIt bindings, including the native contact-map converter."""
    robot_model = pytest.importorskip("moveit.core.robot_model")
    from moveit.core.planning_scene import PlanningScene
    from moveit.core.robot_state import RobotState
    from moveit.core.robot_trajectory import RobotTrajectory
    from moveit_msgs.msg import RobotTrajectory as TrajectoryMessage
    from trajectory_msgs.msg import JointTrajectoryPoint

    urdf = tmp_path / "collision.urdf"
    srdf = tmp_path / "collision.srdf"
    urdf.write_text('''<robot name="diagnostics_test">
      <link name="base"/>
      <link name="trunking"><collision><geometry><box size="0.1 0.1 0.1"/>
        </geometry></collision></link>
      <link name="finger"><collision><geometry><box size="0.1 0.1 0.1"/>
        </geometry></collision></link>
      <joint name="fixture" type="fixed"><parent link="base"/><child link="trunking"/>
        <origin xyz="0 0 0.8"/></joint>
      <joint name="lift" type="prismatic"><parent link="base"/><child link="finger"/>
        <axis xyz="0 0 1"/><limit lower="0" upper="1" effort="1" velocity="1"/></joint>
    </robot>''')
    srdf.write_text('''<robot name="diagnostics_test">
      <group name="arm"><joint name="lift"/></group>
    </robot>''')
    model = robot_model.RobotModel(str(urdf), str(srdf))
    state = RobotState(model)
    state.set_to_default_values()
    state.update()
    scene = PlanningScene(model)
    scene.current_state = state
    trajectory = RobotTrajectory(model)
    trajectory.joint_model_group_name = "arm"
    message = TrajectoryMessage()
    message.joint_trajectory.joint_names = ["lift"]
    message.joint_trajectory.points = [
        JointTrajectoryPoint(positions=[z]) for z in [0.0, 0.8]
    ]
    trajectory.set_robot_trajectory_msg(state, message)
    failure = NS(trajectory=trajectory, start=NS(scene=scene))
    details = _invalid_trajectory_details(
        failure, spec("anchor", group="arm", ik_frame="finger"), NS(anchor_max_path_z=0.6),
    )
    text = "\n".join(details)
    assert "COLLISION at stored waypoint 1/1" in text
    # Older MoveIt bindings cannot convert ContactMap values; their fallback
    # prints the exact colliding links through native FCL's verbose query.
    collision_output = text + capfd.readouterr().err
    assert "finger" in collision_output and "trunking" in collision_output
    assert "HEIGHT_LIMIT: finger z=0.800000 m > anchor_max_path_z=0.600000 m" in text
    assert scene.current_state.joint_positions["lift"] == 0.0
    assert trajectory[1].joint_positions["lift"] == 0.8
