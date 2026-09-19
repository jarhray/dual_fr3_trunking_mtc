"""Terminal orchestration after successful cached MTC execution (simulation only)."""
import copy
import json
import time

from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from std_srvs.srv import Trigger
from dual_fr3_maniskill.usb.insertion import ACTIVE_STATES, SERVICE_OPERATIONS, SERVICE_PREFIX

import dual_fr3_trunking_mtc.insertion_task.motion as insertion_motion
import dual_fr3_trunking_mtc.insertion_task.planning_scene as insertion_planning_scene


class TerminalInsertion:
    """Own service sequencing and temporary MoveIt collision permissions.

    Physics records success and creates retention; this client acknowledges
    retention before detaching the planning object or opening the left hand.
    """

    def __init__(self, client, config_path, node, logger):
        from dual_fr3_maniskill.cable.model import load_geometry_config

        self.client = client
        self.node = node
        self.logger = logger
        self.config_path = config_path
        self.config = load_geometry_config(config_path).get('insertion', {})
        self.services = {
            name: client.node.create_client(Trigger, SERVICE_PREFIX + name)
            for name in SERVICE_OPERATIONS
        }
        self.get_scene = client.node.create_client(GetPlanningScene, '/get_planning_scene')
        self.saved_acm = None
        self.approach_plan = None

    def validate_continuation(self, planned):
        """Reject a transport candidate unless its whole terminal approach plans."""
        from dual_fr3_trunking_mtc.insertion_task.planning import plan_approach

        self.approach_plan = plan_approach(
            self.node, planned.solution.end.scene, self.config, self.logger,
            model_owner=planned)
        return self.approach_plan is not None

    def plan_from_current_state(self):
        """Standalone skill: plan every pre-insertion motion before opening."""
        from dual_fr3_trunking_mtc.insertion_task.planning import plan_approach
        from dual_fr3_trunking_mtc.mtc.preparation_search import capture_start_scene

        self.client.sync_grasp(self.config_path)
        self.sync_socket()
        snapshot = capture_start_scene(self.node)
        self.approach_plan = plan_approach(
            self.node, snapshot.scene, self.config, self.logger,
            model_owner=snapshot.model_owner)
        return self.approach_plan is not None

    def validate_cached_continuation(self, validator):
        """Check the existing approach/preflight from the validated transport end."""
        from dual_fr3_maniskill.cable.model import USB_LINK
        from dual_fr3_maniskill.usb.geometry import SOCKET_NAME, HOLE, measure
        from dual_fr3_maniskill.usb.insertion import InsertionLimits
        from dual_fr3_trunking_mtc.mtc.cached_execution import (
            cache_selected_stages, selected_stage_solution,
        )

        if self.approach_plan is None:
            raise RuntimeError('missing cached insertion approach')
        planned = self.approach_plan.planned
        opening = selected_stage_solution(planned, 'preview_right_release')
        if not validator.validate('preview_right_release', opening, gripper=True):
            return False
        for item in cache_selected_stages(planned):
            if item.spec.name == 'insertion_collision_preflight':
                # Match the original preflight's sole contact exception. This
                # changes only the validator's copy, never the live scene.
                validator.scene.allowed_collision_matrix.set_entry(USB_LINK, SOCKET_NAME, True)
            if not validator.validate(item.spec.name, item.solution):
                return False
            if item.spec.name == 'socket_preinsert':
                limits = InsertionLimits.read(self.config)
                observation = measure(
                    validator.scene.get_frame_transform(SOCKET_NAME),
                    validator.scene.get_frame_transform(USB_LINK),
                    self.config.get('hole_center_m', HOLE))
                if (abs(observation['depth_m'] + limits.preinsert_m) > limits.tracking_limit_m or
                        observation['lateral_error_m'] > limits.lateral_tolerance_m or
                        observation['orientation_error_rad'] > limits.angle_tolerance_rad):
                    self.logger.error('[cached-validation] socket_preinsert: measured grasp fails original insertion alignment limits')
                    return False
        return True

    def command(self, operation):
        response = self.client._call(self.services[operation], Trigger.Request())
        return json.loads(response.message)

    def apply(self, scene):
        self.client._call(self.client.apply, ApplyPlanningScene.Request(scene=scene))

    def collision_permissions(self):
        response = self.client._call(
            self.get_scene,
            GetPlanningScene.Request(components=PlanningSceneComponents(
                components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)),
            require_success=False,
        )
        return response.scene.allowed_collision_matrix

    def sync_socket(self):
        socket = insertion_planning_scene.socket_collision_object(
            self.config, self.command('status'))
        self.apply(insertion_planning_scene.socket_scene(socket, self.collision_permissions()))

    def motion(self, name, group, target, *, cartesian=False, link=None, execute=True):
        return insertion_motion.motion(
            self.node, self.config, self.logger, name, group, target,
            cartesian=cartesian, link=link, execute=execute)

    def withdraw(self, side):
        return insertion_motion.withdraw(self.node, self.config, side)

    def detach_measured_usb(self, status):
        scene = insertion_planning_scene.detached_usb_scene(self.config_path, status)
        acm = self.collision_permissions()
        self.saved_acm = copy.deepcopy(acm)
        insertion_planning_scene.allow_grasp_contacts(scene, acm)
        self.apply(scene)

    def restore_collision_permissions(self):
        if self.saved_acm is not None:
            self.apply(PlanningScene(is_diff=True, allowed_collision_matrix=self.saved_acm))
            self.saved_acm = None

    def open_gripper(self, gripper_controller, side):
        from dual_fr3_trunking_mtc.execution.gripper import GripperRequest

        actor = 'follower' if side == 'right' else 'leader'
        default_width = .03 if side == 'right' else .08
        width = float(self.config.get('release_width_m', {}).get(side, default_width))
        if not gripper_controller.execute(
                GripperRequest(actor=actor, profile='open', width_override=width)):
            raise RuntimeError(side + ' gripper release failed')

    def release_right(self, gripper_controller):
        self.command('right_release')
        self.open_gripper(gripper_controller, 'right')
        self.command('right_released')

    def return_right(self):
        self.approach_plan.execute('withdraw_right')
        self.approach_plan.execute('right_return_ready')
        self.command('right_returned')

    def align_left(self):
        self.command('target')  # existing right_ready -> approach handoff
        if float(self.config.get('approach_lift_m', 0.)):
            self.approach_plan.execute('socket_above_approach')
        self.approach_plan.execute('socket_preinsert')
        # start observes the actual USB and applies the existing alignment
        # limits. Grasp drift stops here; a cached pose is never proof of alignment.

    def feedback_insert(self):
        self.command('start')
        wall_start = time.monotonic()
        while True:
            # Only this explicit lease renews ownership; status is read-only.
            status = self.command('heartbeat')
            if status['state'] not in ('aligned', *ACTIVE_STATES):
                break
            if time.monotonic() - wall_start > max(120., 30 * self.config.get('timeout_s', 45.)):
                raise RuntimeError('Terminal insertion wall-time watchdog')
            time.sleep(.1)
        self.logger.info('insertion result: %s', json.dumps(status))
        if not status['insertion_success']:
            raise RuntimeError('Insertion failed: ' + status['state'] + ' ' + status['reason'])

    def release_left(self, gripper_controller, status):
        self.detach_measured_usb(status)
        self.open_gripper(gripper_controller, 'left')
        self.command('released')

    def return_left(self):
        self.command('returning')
        self.withdraw('left')
        self.restore_collision_permissions()
        self.motion('left_return_ready', 'left_fr3_arm', 'ready')
        self.command('returned')

    def execute(self, gripper_controller):
        retained = False
        released = False
        phase = 'initial'
        try:
            status = self.command('status')
            if status['return_complete']:
                return True
            if status['state'] != 'not_started':
                raise RuntimeError('Terminal operation already attempted; reset before replay')
            if self.approach_plan is None and not self.plan_from_current_state():
                raise RuntimeError('Complete insertion approach is not plannable')

            phase = 'right_release'
            self.release_right(gripper_controller)
            phase = 'right_return'
            self.return_right()
            phase = 'insertion'
            self.align_left()
            self.feedback_insert()

            if not self.config.get('retain_after_success', True):
                return True
            status = self.command('retained')
            retained = True
            if not self.config.get('release_after_retention', True):
                return True
            self.release_left(gripper_controller, status)
            released = True
            if not self.config.get('return_after_release', True):
                return True
            self.return_left()
            return True
        except Exception as exc:
            self.logger.error('terminal insertion stopped: %s', exc)
            if phase in ('right_release', 'right_return'):
                try:
                    self.command('fail_' + phase)
                except Exception:
                    self.logger.exception('Unable to report right-arm preparation failure')
            elif retained:
                try:
                    self.command('fail_return' if released else 'fail_release')
                except Exception:
                    self.logger.exception('Unable to report return failure')
            return False
        finally:
            try:
                self.command('cancel')
            except Exception:
                self.logger.exception('Unable to acknowledge controller release')
            if self.saved_acm is not None:
                try:
                    self.apply(PlanningScene(is_diff=True, allowed_collision_matrix=self.saved_acm))
                except Exception:
                    self.logger.exception('Unable to restore grasp collision pairs')
