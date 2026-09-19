"""Validate cached joint paths in the measured scene without motion planning."""

import copy
import math

from dual_fr3_trunking_mtc.mtc.cached_execution import selected_stage_solution
from dual_fr3_trunking_mtc.mtc.preparation_search import capture_start_scene


# Match MoveIt's default execution start tolerance. Check every stored waypoint
# and subdivide larger gaps; this is discrete validation, not continuous CCD.
START_TOLERANCE = 0.01
MAX_JOINT_STEP = 0.002  # rad for revolute joints, m for prismatic joints


class CachedPathValidator:
    def __init__(self, scene, logger):
        self.scene = copy.copy(scene)
        self.state = copy.copy(scene.current_state)
        self.state.update()
        self.logger = logger
        self.attachments = copy.deepcopy(
            scene.planning_scene_message.robot_state.attached_collision_objects)
        self.solutions = []
        self.bounds = {}
        for group in scene.robot_model.joint_model_groups:
            for name, bounds in zip(group.active_joint_model_names, group.active_joint_model_bounds):
                if len(bounds) != 1:
                    raise ValueError('cached validation supports single-variable joints only')
                self.bounds[name] = bounds[0]

    def _valid_state(self, name, location):
        self.state.update()
        for joint, value in self.state.joint_positions.items():
            bound = self.bounds.get(joint)
            if not math.isfinite(value) or (bound is not None and bound.position_bounded and
                    not bound.min_position - 1.e-6 <= value <= bound.max_position + 1.e-6):
                self.logger.error('[cached-validation] %s %s: joint %s out of bounds (%s)',
                                  name, location, joint, value)
                return False
        # Empty group checks both arms, fingers and the *measured* attachment.
        if not self.scene.is_state_valid(self.state, ''):
            self.logger.error('[cached-validation] %s %s: collision or infeasible state; native details follow',
                              name, location)
            self.scene.is_state_valid(self.state, '', True)
            return False
        return True

    def validate(self, name, solution, *, gripper=False):
        trajectory = getattr(solution, 'trajectory', None)
        if trajectory is None or len(trajectory) == 0:
            raise ValueError(f'{name}: missing cached trajectory')
        message = trajectory.get_robot_trajectory_msg()
        if message.multi_dof_joint_trajectory.points:
            raise ValueError(f'{name}: unsupported multi-DOF cached trajectory')
        names = list(message.joint_trajectory.joint_names)
        points = message.joint_trajectory.points
        if not names or not points or len(set(names)) != len(names):
            raise ValueError(f'{name}: malformed cached trajectory')
        current = self.state.joint_positions
        if any(joint not in current or joint not in self.bounds for joint in names):
            raise ValueError(f'{name}: unknown cached joint')
        for point in points:
            if len(point.positions) != len(names) or not all(map(math.isfinite, point.positions)):
                raise ValueError(f'{name}: invalid cached joint positions')
        if not gripper:
            for joint, target in zip(names, points[0].positions):
                error = abs(current[joint] - target)
                if error > START_TOLERANCE:
                    self.logger.error('[cached-validation] %s: start mismatch %s=%.6f > %.6f',
                                      name, joint, error, START_TOLERANCE)
                    return False
        if not self._valid_state(name, 'start'):
            return False
        # A profile-based gripper action starts at its actual contact width,
        # not the nominal closed width in the planning preview.
        targets = points[-1:] if gripper else points
        samples = 0
        for index, point in enumerate(targets):
            start = [self.state.joint_positions[joint] for joint in names]
            count = max(1, math.ceil(max(abs(b - a) for a, b in zip(start, point.positions)) / MAX_JOINT_STEP))
            for step in range(1, count + 1):
                self.state.joint_positions = {
                    joint: a + (b - a) * step / count
                    for joint, a, b in zip(names, start, point.positions)
                }
                samples += 1
                if not self._valid_state(name, f'waypoint {index}, sample {step}/{count}'):
                    return False
        self.scene.current_state = self.state
        self.solutions.append(solution)
        self.logger.info('[cached-validation] %s: accepted (%d samples)', name, samples)
        return True

    def refresh_attachments(self):
        """Keep outgoing MTC scene effects from restoring the nominal grasp.

        Native SubTrajectory serialization includes attachments in its end-scene
        diff. Refresh only those scene attachments, after all paths pass; leave
        the selected RobotTrajectory (positions, velocities and timing) intact.
        """
        for solution in self.solutions:
            for scene in (solution.start.scene, solution.end.scene):
                for attachment in self.attachments:
                    if not scene.process_attached_collision_object(attachment):
                        raise RuntimeError('cannot refresh cached scene attachment ' + attachment.object.id)


def validate_cached_after_grasp(node, planned, cached, logger, *, continuation_validator=None):
    # CurrentState only captures the scene; it does not invoke a motion planner.
    # Retain the task/model loader while the native scene is in use.
    snapshot = capture_start_scene(node)
    validator = CachedPathValidator(snapshot.scene, logger)
    from dual_fr3_maniskill.cable.model import USB_LINK
    if not any(obj.object.id == USB_LINK for obj in validator.attachments):
        raise RuntimeError('measured planning scene has no USB attachment')
    for item in cached:
        if item.spec.mtc_stage_type == 'SimulationCable':
            raise ValueError('unexpected simulation scene operation after grasp')
        solution = item.solution or selected_stage_solution(planned, item.spec.name)
        if not validator.validate(item.spec.name, solution,
                                  gripper=item.spec.mtc_stage_type == 'GripperOperation'):
            return False
    if continuation_validator is not None and not continuation_validator(validator):
        return False
    validator.refresh_attachments()
    return True
