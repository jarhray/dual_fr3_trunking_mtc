"""Exercise TCP path checks with real MoveIt FK and trajectory bindings."""

import logging
import math
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory as TrajectoryMessage
from trajectory_msgs.msg import JointTrajectoryPoint

from dual_fr3_trunking_mtc.mtc.path_length import (
    anchor_path_length_cost,
    tcp_path_length,
    terminal_path_length_cost,
    validate_path_length_ratio,
)
from dual_fr3_trunking_mtc.mtc.cartesian_validation import (
    cartesian_path_cost,
    merged_cartesian_path_cost,
)
from dual_fr3_trunking_mtc.mtc.diagnostics import explain_cost_failure


@pytest.fixture
def model(tmp_path):
    robot_model = pytest.importorskip("moveit.core.robot_model")
    urdf = tmp_path / "arm.urdf"
    srdf = tmp_path / "arm.srdf"
    urdf.write_text('''<robot name="length_test">
      <link name="base"/><link name="rotor"/><link name="tcp"/>
      <link name="target_frame"/><link name="peer_tcp"/>
      <joint name="joint" type="revolute">
        <parent link="base"/><child link="rotor"/><axis xyz="0 0 1"/>
        <limit lower="-3.15" upper="3.15" effort="1" velocity="1"/>
      </joint>
      <joint name="tool" type="fixed">
        <parent link="rotor"/><child link="tcp"/><origin xyz="1 0 0"/>
      </joint>
      <joint name="frame" type="fixed">
        <parent link="base"/><child link="target_frame"/>
        <origin xyz="2 3 4" rpy="0 0 1.5707963267948966"/>
      </joint>
      <joint name="peer_joint" type="prismatic">
        <parent link="base"/><child link="peer_tcp"/><axis xyz="1 0 0"/>
        <limit lower="0" upper="1" effort="1" velocity="1"/>
      </joint>
    </robot>''')
    srdf.write_text('''<robot name="length_test">
      <group name="arm"><chain base_link="base" tip_link="tcp"/></group>
      <group name="peer"><joint name="peer_joint"/></group>
      <group_state name="half_turn" group="arm"><joint name="joint" value="3.141592653589793"/></group_state>
    </robot>''')
    return robot_model.RobotModel(str(urdf), str(srdf))


def make_trajectory(model, angles):
    from moveit.core.robot_state import RobotState
    from moveit.core.robot_trajectory import RobotTrajectory

    state = RobotState(model)
    state.set_to_default_values()
    state.joint_positions = {"joint": angles[0] if angles else 0.0}
    state.update()
    message = TrajectoryMessage()
    message.joint_trajectory.joint_names = ["joint"]
    message.joint_trajectory.points = [
        JointTrajectoryPoint(positions=[angle]) for angle in angles
    ]
    trajectory = RobotTrajectory(model)
    trajectory.joint_model_group_name = "arm"
    trajectory.set_robot_trajectory_msg(state, message)
    return trajectory


def make_solution(model, angles):
    from moveit.core.planning_scene import PlanningScene

    trajectory = make_trajectory(model, angles)
    scene = PlanningScene(model)
    if angles:
        scene.current_state = trajectory[0]
    return SimpleNamespace(
        trajectory=trajectory, start=SimpleNamespace(scene=scene), comment=""
    )


def target_at(angle, frame="base"):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = math.cos(angle)
    pose.pose.position.y = math.sin(angle)
    pose.pose.orientation.w = 1.0
    return pose


def test_sparse_joint_trajectory_measures_arc_not_endpoint_chord(model):
    trajectory = make_trajectory(model, [0.0, math.pi])
    before = [trajectory[i].joint_positions for i in range(len(trajectory))]

    assert tcp_path_length(trajectory, "tcp") == pytest.approx(math.pi, abs=4e-6)
    assert [trajectory[i].joint_positions for i in range(len(trajectory))] == before
    assert len(trajectory) == 2


def test_cartesian_gate_detects_excursion_between_nearby_tcp_waypoints(model, caplog):
    # The two TCP samples are only 2 cm apart, but bounded joint interpolation
    # from -179 to +179 degrees makes almost a full revolution between them.
    solution = make_solution(model, [-math.pi + 0.01, math.pi - 0.01])
    before = [solution.trajectory[i].joint_positions for i in range(2)]
    cost = cartesian_path_cost("tcp", 0.005, "straight")
    assert math.isinf(cost(solution))
    assert "line deviation 1.9999" in caplog.text
    assert "REJECTED" in caplog.text
    assert "CARTESIAN_PATH_DEVIATION" in solution.comment
    assert "cartesian_path_tolerance=0.005000 m" in solution.comment
    assert [solution.trajectory[i].joint_positions for i in range(2)] == before


def test_cartesian_gate_accepts_small_interpolation_error(model, caplog):
    caplog.set_level(logging.INFO)
    cost = cartesian_path_cost("tcp", 0.005, "straight")
    assert math.isfinite(cost(make_solution(model, [0.0, 0.1])))
    assert "accepted" in caplog.text


def test_in_place_gate_checks_motion_not_just_matching_endpoints(model):
    solution = make_solution(model, [0.0, math.pi / 2, 0.0])
    assert math.isinf(cartesian_path_cost("tcp", 0.005, "turn", stationary=True)(solution))
    # The rotation-axis frame changes orientation while its position stays fixed.
    assert cartesian_path_cost("rotor", 0.005, "turn", stationary=True)(solution) == 0.0


def test_cartesian_gate_rejects_empty_or_unknown_tcp(model):
    assert math.isinf(cartesian_path_cost("tcp", 0.005, "straight")(make_solution(model, [])))
    assert math.isinf(cartesian_path_cost("missing", 0.005, "straight")(
        make_solution(model, [0.0, 0.1])
    ))


@pytest.mark.parametrize("angle,accepted", [(0.1, True), (math.pi, False)])
@pytest.mark.parametrize("merged", [False, True])
def test_real_mtc_cartesian_gate_blocks_detours(model, angle, accepted, merged):
    """Even a collision-free candidate reaching its goal must pass the TCP gate."""
    import rclcpp
    from moveit.core.planning_scene import PlanningScene
    from moveit.task_constructor import core, stages

    rclcpp.init()
    try:
        task = core.Task()
        task.setRobotModel(model)
        scene = PlanningScene(model)
        scene.current_state = make_trajectory(model, [0.0])[0]
        start = stages.FixedState("start")
        start.setState(scene)
        task.add(start)
        move = stages.MoveTo("move", core.JointInterpolationPlanner())
        move.group = "arm"
        move.setGoal({"joint": angle})
        if merged:
            container = core.Merger("merged")
            container.insert(move)
            peer = stages.MoveTo("peer", core.JointInterpolationPlanner())
            peer.group = "peer"
            peer.setGoal({"peer_joint": 0.1})
            container.insert(peer)
            container.setCostTerm(merged_cartesian_path_cost(["tcp", "peer_tcp"], 0.005, "merged"))
            task.add(container)
        else:
            move.setCostTerm(cartesian_path_cost("tcp", 0.005, "move"))
            task.add(move)
        assert bool(task.plan()) is accepted
        assert bool(task.solutions) is accepted
        if not accepted:
            failed_stage = task["merged"] if merged else task["move"]
            spec = SimpleNamespace(
                name=failed_stage.name, primitive="cartesian_translate", planner="CartesianPath",
                mtc_stage_type="Merger" if merged else "MoveRelative", ik_frame="tcp",
                children=[SimpleNamespace(ik_frame=link) for link in ("tcp", "peer_tcp")],
            )
            explain_cost_failure(
                failed_stage.failures[0], spec, None,
                SimpleNamespace(cartesian_path_tolerance=0.005),
            )
            assert "CARTESIAN_PATH_DEVIATION" in failed_stage.failures[0].comment
    finally:
        rclcpp.shutdown()


@pytest.mark.parametrize("angle,accepted", [(math.pi / 2, True), (math.pi, False)])
def test_ratio_accepts_short_path_and_rejects_detour(model, angle, accepted, caplog):
    caplog.set_level(logging.INFO)
    solution = make_solution(model, [0.0, angle])
    cost = anchor_path_length_cost("tcp", target_at(angle), 1.5, "anchor")

    assert math.isfinite(cost(solution)) is accepted
    assert ("REJECTED" in caplog.text) is not accepted


def test_distance_uses_actual_start_and_transforms_requested_target(model, caplog):
    caplog.set_level(logging.INFO)
    solution = make_solution(model, [math.pi / 2, math.pi])
    target = target_at(math.pi, "target_frame")
    # base (-1, 0, 0) expressed in a translated and rotated target frame.
    target.pose.position.x = -3.0
    target.pose.position.y = 3.0
    target.pose.position.z = -4.0
    cost = anchor_path_length_cost("tcp", target, 1.5, "anchor")

    assert cost(solution) == pytest.approx(math.pi / 2, abs=2e-6)
    assert "start-to-target 1.414214 m" in caplog.text


def test_requested_target_not_achieved_endpoint_defines_limit(model):
    solution = make_solution(model, [0.0, math.pi / 2])
    # Requested target coincides with the start; an incorrect endpoint must not
    # enlarge the budget. MoveTo separately validates the endpoint pose.
    cost = anchor_path_length_cost("tcp", target_at(0.0), 1.5, "anchor")
    assert math.isinf(cost(solution))


def test_coincident_endpoints_reject_excursion_but_accept_stationary(model):
    cost = anchor_path_length_cost("tcp", target_at(0.0), 1.5, "anchor")
    assert math.isinf(cost(make_solution(model, [0.0, math.pi / 2, 0.0])))
    assert cost(make_solution(model, [0.0, 0.0])) == 0.0


@pytest.mark.parametrize("ratio", [0.0, 0.99, -1.0, math.nan, math.inf])
def test_invalid_ratios_are_rejected(ratio):
    with pytest.raises(ValueError, match="anchor_max_path_length_ratio"):
        validate_path_length_ratio(ratio)


def test_uncheckable_paths_fail_closed(model, caplog):
    cost = anchor_path_length_cost("tcp", target_at(0.0), 1.5, "anchor")
    assert math.isinf(cost(make_solution(model, [])))
    solution = make_solution(model, [0.0, math.pi / 2])
    unknown_frame_cost = anchor_path_length_cost(
        "tcp", target_at(math.pi / 2, "missing_frame"), 1.5, "anchor"
    )
    assert math.isinf(unknown_frame_cost(solution))
    assert "unknown frame" in caplog.text


@pytest.mark.parametrize("angle,accepted", [(math.pi / 2, True), (math.pi, False)])
def test_real_mtc_callback_filters_solutions(model, angle, accepted, caplog):
    """Verify cost callbacks actually block propagation in installed MTC."""
    import rclcpp
    from moveit.core.planning_scene import PlanningScene
    from moveit.task_constructor import core, stages

    caplog.set_level(logging.INFO)
    rclcpp.init()
    try:
        task = core.Task()
        task.setRobotModel(model)
        scene = PlanningScene(model)
        scene.current_state = make_trajectory(model, [0.0])[0]
        start = stages.FixedState("start")
        start.setState(scene)
        move = stages.MoveTo("anchor", core.JointInterpolationPlanner())
        move.group = "arm"
        move.setGoal({"joint": angle})
        move.setCostTerm(anchor_path_length_cost("tcp", target_at(angle), 1.5, "anchor"))
        task.add(start)
        task.add(move)

        assert bool(task.plan()) is accepted
        assert bool(task.solutions) is accepted
        if accepted:
            assert task["anchor"].solutions[0].cost == pytest.approx(angle, abs=2e-6)
            assert "accepted" in caplog.text
        else:
            assert not task["anchor"].solutions
            assert "REJECTED" in caplog.text
            explain_cost_failure(
                task["anchor"].failures[0],
                SimpleNamespace(name="anchor", primitive="direct_move_to_next_anchor",
                                to_index=0, ik_frame="tcp"),
                SimpleNamespace(keypoints=[SimpleNamespace(
                    frame_id="base", position=(math.cos(angle), math.sin(angle), 0.0),
                )]),
                SimpleNamespace(anchor_max_path_length_ratio=1.5),
            )
            assert "PATH_LENGTH_LIMIT: TCP path" in task["anchor"].failures[0].comment
            assert "limit 3.000000 m" in task["anchor"].failures[0].comment
    finally:
        rclcpp.shutdown()


def test_cache_matches_real_mtc_full_solution_and_captures_absolute_goal(model):
    import rclcpp
    from moveit.core.planning_scene import PlanningScene
    from moveit.task_constructor import core, stages
    from dual_fr3_trunking_mtc.mtc.cached_execution import (
        cache_selected_stages, capture_relative_targets,
    )
    from dual_fr3_trunking_mtc.mtc.planning import PlannedTask

    rclcpp.init()
    try:
        task = core.Task()
        task.setRobotModel(model)
        scene = PlanningScene(model)
        scene.current_state = make_trajectory(model, [0.0])[0]
        start = stages.FixedState("start")
        start.setState(scene)
        task.add(start)
        specs = []
        for index, angle in enumerate((0.5, 1.0)):
            name = f"stage_{index}"
            move = stages.MoveTo(name, core.JointInterpolationPlanner())
            move.group = "arm"
            move.setGoal({"joint": angle})
            task.add(move)
            specs.append(SimpleNamespace(
                name=name, executable=True, mtc_stage_type="MoveRelative", ik_frame="tcp",
            ))
        assert task.plan()
        cached = cache_selected_stages(PlannedTask(task, task.solutions[0], tuple(specs)))
        assert len(cached) == 2
        assert cached[0].solution.end.scene.current_state.joint_positions["joint"] == 0.5
        assert cached[1].solution.start.scene.current_state.joint_positions["joint"] == 0.5
        goals = capture_relative_targets(cached)
        assert goals["stage_1"].header.frame_id == "base"
        assert goals["stage_1"].pose.position.x == pytest.approx(math.cos(1.0))
        assert goals["stage_1"].pose.position.y == pytest.approx(math.sin(1.0))
    finally:
        rclcpp.shutdown()


@pytest.mark.parametrize('kind', ['pose','named','relative'])
@pytest.mark.parametrize('ratio,accepted', [(1.5,False),(2.,True)])
def test_terminal_motion_goals_share_dense_tcp_path_gate(model, kind, ratio, accepted):
    from geometry_msgs.msg import Vector3Stamped, Vector3
    from std_msgs.msg import Header
    targets = dict(pose=target_at(math.pi), named='half_turn',
                   relative=Vector3Stamped(header=Header(frame_id='base'),vector=Vector3(x=-2.)))
    solution = make_solution(model,[0.,math.pi])
    callback = terminal_path_length_cost('tcp', targets[kind], ratio, 'terminal', group='arm')
    assert math.isfinite(callback(solution)) is accepted
    assert solution.trajectory[0].joint_positions['joint'] == 0.
    assert solution.trajectory[len(solution.trajectory)-1].joint_positions['joint'] == math.pi


def test_terminal_relative_goal_is_resolved_in_requested_frame(model):
    from geometry_msgs.msg import Vector3Stamped, Vector3
    from std_msgs.msg import Header
    # target_frame rotates +90 degrees: local +Y is model -X.
    target = Vector3Stamped(header=Header(frame_id='target_frame'),vector=Vector3(y=2.))
    callback = terminal_path_length_cost('tcp',target,2.,'withdraw')
    assert callback(make_solution(model,[0.,math.pi])) == pytest.approx(math.pi,abs=4e-6)


@pytest.mark.parametrize('ratio,accepted',[(1.5,False),(2.,True)])
def test_native_mtc_rejects_terminal_named_goal_detour_before_execution(model, ratio, accepted):
    import rclcpp
    from moveit.core.planning_scene import PlanningScene
    from moveit.task_constructor import core, stages
    rclcpp.init()
    try:
        task = core.Task(introspection=False)
        task.setRobotModel(model)
        scene = PlanningScene(model)
        scene.current_state = make_trajectory(model,[0.])[0]
        start = stages.FixedState('start'); start.setState(scene); task.add(start)
        move = stages.MoveTo('terminal_return', core.JointInterpolationPlanner())
        move.group = 'arm'; move.setGoal('half_turn')
        move.setCostTerm(terminal_path_length_cost('tcp','half_turn',ratio,'terminal_return',group='arm'))
        task.add(move)
        assert bool(task.plan(1)) is accepted
        assert bool(task.solutions) is accepted
    finally:
        rclcpp.shutdown()
