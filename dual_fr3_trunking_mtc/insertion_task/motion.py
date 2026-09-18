"""Terminal MTC motion construction, planning retries and single execution.

Every retry samples from measured CurrentState. Execution is attempted once;
a partially executed trajectory must never be blindly replayed.
"""

def motion(node, config, logger, name, group, target, *, cartesian=False, link=None, execute=True):
    from dual_fr3_trunking_mtc.mtc.task_builder import (
        import_mtc_modules,
        create_motion_planners,
        _identity_ik_frame,
    )
    from dual_fr3_trunking_mtc.mtc.path_length import terminal_path_length_cost
    _, core, stages = import_mtc_modules()
    attempts = int(config.get('planning_attempts', 10))
    for attempt in range(1, attempts+1):
        # A failed plan has issued no motion. Reconstruct CurrentState and
        # the pipeline so each OMPL search can obtain a fresh sample set.
        task = core.Task()
        task.name = name
        task.loadRobotModel(node)
        task.add(stages.CurrentState('measured_state'))
        if not execute:
            expected = stages.ModifyPlanningScene('expected_USB_socket_contact_only')
            expected.allowCollisions('usb_cable_demo_plug', ['usb_socket'], True)
            task.add(expected)
        cart, _, ompl = create_motion_planners(core, node, .001, .08, .08, 2.)
        move = stages.MoveTo(name, cart if cartesian else ompl)
        move.group = group
        if link:
            move.ik_frame = _identity_ik_frame(link)
        move.setGoal(target)
        move.setCostTerm(terminal_path_length_cost(
            link or group.removesuffix('_arm')+'_hand_tcp', target,
            float(config.get('max_path_length_ratio', 1.5)), name, group=group))
        move.timeout = float(config.get('planning_timeout_s', 15.))
        task.add(move)
        logger.info('%s planning attempt %d/%d', name, attempt, attempts)
        if task.plan(1) and task.solutions:
            break
    else:
        raise RuntimeError(name+': motion planning exhausted '+str(attempts)+' attempts')
    if not execute:
        return
    # Never blindly replay a trajectory after a partially executed failure.
    result = task.execute(task.solutions[0])
    if not result:
        raise RuntimeError(name+': motion execution failed '+str(getattr(result, 'val', result)))
    logger.info('terminal insertion motion completed: %s', name)


def withdraw(node, config, side):
    from moveit.task_constructor import core, stages
    from geometry_msgs.msg import Vector3Stamped, Vector3
    from std_msgs.msg import Header
    from dual_fr3_trunking_mtc.mtc.task_builder import create_motion_planners, _identity_ik_frame
    from dual_fr3_trunking_mtc.mtc.path_length import terminal_path_length_cost
    attempts = int(config.get('planning_attempts', 10))
    for attempt in range(attempts):
        task = core.Task()
        task.name = side+'_socket_withdraw'
        task.loadRobotModel(node)
        task.add(stages.CurrentState('measured_open_state'))
        cart, _, _ = create_motion_planners(core, node, .001, .05, .05, 2.)
        move = stages.MoveRelative('withdraw_open_'+side, cart)
        move.group = side+'_fr3_arm'
        link = side+'_fr3_hand_tcp'
        move.ik_frame = _identity_ik_frame(link)
        direction = Vector3Stamped(header=Header(frame_id=link),
            vector=Vector3(z=-float(config.get('retreat_m', .05))))
        move.setDirection(direction)
        move.setCostTerm(terminal_path_length_cost(link, direction,
            float(config.get('max_path_length_ratio', 1.5)), side+'_socket_withdraw'))
        task.add(move)
        if task.plan(1) and task.solutions:
            break
    else:
        raise RuntimeError(side+' withdrawal planning failed')
    if not task.execute(task.solutions[0]):
        raise RuntimeError(side+' withdrawal execution failed')
