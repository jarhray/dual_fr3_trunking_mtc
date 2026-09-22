"""Opt-in real insertion services. Imports ROS only when the node is started."""
import argparse
from collections import OrderedDict
import json
from pathlib import Path
import threading
import time

import numpy as np
from transforms3d.quaternions import quat2mat

from .local_kinematics import LocalKinematics, pose_error
from .real_session import RealInsertionSession, load_real_config, config_fingerprint


def pose_matrix(pose):
    q = np.array([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
    if not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1.) > .001:
        raise ValueError('Invalid robot pose quaternion')
    result = np.eye(4)
    result[:3, :3] = quat2mat(q)
    result[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    if not np.isfinite(result).all():
        raise ValueError('Nonfinite robot pose')
    return result


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1.e-9


def _json(value):
    return json.dumps(value, allow_nan=False, default=lambda x: x.tolist() if isinstance(x, np.ndarray) else str(x))


def make_node(config, allow_execution=False):
    import rclpy
    from rclpy.node import Node
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from sensor_msgs.msg import JointState
    from franka_msgs.msg import FrankaRobotState
    from controller_manager_msgs.srv import SwitchController, ListControllers
    from rcl_interfaces.srv import GetParameters
    from moveit_msgs.srv import GetStateValidity, GetPlanningScene
    from moveit_msgs.msg import PlanningSceneComponents
    from dual_fr3_moveit_config.msg import JointTarget, JointTargetState as State
    from dual_fr3_maniskill.usb.geometry import pose_dict, tcp_goal
    from dual_fr3_maniskill.usb.insertion import SERVICE_OPERATIONS, ALIGNMENT_STATES

    class RealInsertionNode(Node):
        def __init__(self):
            super().__init__('dual_fr3_real_insertion')
            if self.get_parameter('use_sim_time').value:
                raise RuntimeError('Real insertion cannot use simulation time')
            self.cfg, self.settings = config, config['real_runtime']
            self.session = RealInsertionSession(config)
            self.allow_execution = allow_execution
            self.group = ReentrantCallbackGroup()
            self.lock = threading.Lock()
            self.robot = self.gripper = self.all_joints = self.controller = None
            self.right_robot = self.right_gripper = None
            self.kinematics = None
            self.owner = self.running = False
            self.sequence = 0
            self.controller_session = None
            self.q_reference = None
            self.last_target_stamp = None
            self.last_target_q = None
            self.executor_limits = None
            self.switch_requested = self.ownership_uncertain = False
            self.controller_inventory = {}
            self.stop_requested = self.stop_acknowledged = False
            self.stop_service_accepted = False
            self.stop_future = None
            self.stop_requested_at = None
            self.executor_failure = None
            self.heartbeat = time.monotonic()
            self.names = [f'left_fr3_joint{i}' for i in range(1, 8)]
            self.last_robot_time = None
            self.source_times = OrderedDict()
            self.joint_names = []
            self.publisher = self.create_publisher(String, '/real/usb/insertion_state', 10)
            ctl = self.settings.get('controller_path', '/left/left_insertion_controller')
            manager = self.settings.get('controller_manager', '/left/controller_manager')
            self.targets = self.create_publisher(JointTarget, ctl + '/target', 1)
            self.stop_client = self.create_client(Trigger, ctl + '/stop', callback_group=self.group)
            self.switch_client = self.create_client(SwitchController, manager + '/switch_controller', callback_group=self.group)
            self.list_client = self.create_client(ListControllers, manager + '/list_controllers', callback_group=self.group)
            self.parameters_client = self.create_client(GetParameters, ctl + '/get_parameters', callback_group=self.group)
            self.description_client = self.create_client(GetParameters, '/robot_state_publisher/get_parameters', callback_group=self.group)
            self.validity_client = self.create_client(GetStateValidity, '/check_state_validity', callback_group=self.group)
            self.scene_client = self.create_client(GetPlanningScene, '/get_planning_scene', callback_group=self.group)
            self.create_subscription(FrankaRobotState,
                self.settings.get('robot_state_topic', '/left/left_franka_robot_state_broadcaster/robot_state'),
                self.receive_robot, qos_profile_sensor_data, callback_group=self.group)
            self.create_subscription(JointState, self.settings.get('gripper_topic', '/left_franka_gripper/joint_states'),
                lambda m: setattr(self, 'gripper', (m, time.monotonic())), qos_profile_sensor_data, callback_group=self.group)
            self.create_subscription(JointState, '/joint_states',
                lambda m: setattr(self, 'all_joints', (m, time.monotonic())), qos_profile_sensor_data, callback_group=self.group)
            self.create_subscription(JointState,
                self.settings.get('right_robot_joint_topic', '/right/franka/joint_states'),
                lambda m: setattr(self, 'right_robot', (m, time.monotonic())), qos_profile_sensor_data, callback_group=self.group)
            self.create_subscription(JointState,
                self.settings.get('right_gripper_topic', '/right_franka_gripper/joint_states'),
                lambda m: setattr(self, 'right_gripper', (m, time.monotonic())), qos_profile_sensor_data, callback_group=self.group)
            self.create_subscription(State, ctl + '/state',
                lambda m: setattr(self, 'controller', (m, time.monotonic())), 1, callback_group=self.group)
            prefix = self.settings.get('service_prefix', '/real/usb/insertion/')
            self.services = [self.create_service(Trigger, prefix + op,
                lambda req, res, op=op: self.command(op, res), callback_group=self.group)
                for op in SERVICE_OPERATIONS]
            self.create_timer(1. / self.settings['frequency_hz'], self.tick, callback_group=self.group)
            self.log = None
            if self.settings.get('record_path'):
                self.log = open(Path(self.settings['record_path']).expanduser(), 'a', encoding='utf-8')
                self.session.record_policy_event = self.record_policy_event

        def record_policy_event(self, event):
            if self.log:
                self.log.write(_json(event) + '\n')
                self.log.flush()

        def receive_robot(self, message):
            # Duplicate robot samples must not renew freshness just because DDS
            # keeps delivering them. A rollback invalidates the session.
            if self.last_robot_time is not None and message.time <= self.last_robot_time:
                if message.time < self.last_robot_time:
                    self.robot = None
                return
            self.last_robot_time = message.time
            self.robot = (message, time.monotonic())

        def call(self, client, request, timeout=2.):
            if not client.wait_for_service(timeout_sec=min(timeout, .5)):
                raise RuntimeError('Unavailable service: ' + client.srv_name)
            done = threading.Event()
            future = client.call_async(request)
            future.add_done_callback(lambda _: done.set())
            if not done.wait(timeout):
                raise RuntimeError('Service timeout, operation not retried: ' + client.srv_name)
            result = future.result()
            if result is None:
                raise RuntimeError('Empty service response: ' + client.srv_name)
            return result

        def ensure_kinematics(self):
            if self.kinematics is None:
                reply = self.call(self.description_client, GetParameters.Request(names=['robot_description']))
                if not reply.values or not reply.values[0].string_value:
                    raise RuntimeError('No robot_description')
                self.kinematics = LocalKinematics(reply.values[0].string_value, joint_names=self.names)

        def message_time(self, message, received):
            # Map a source timestamp once on first use; cached messages retain
            # their original monotonic time across status/control reads.
            stamp = stamp_seconds(message.header.stamp)
            key = (type(message).__name__, message.header.frame_id, stamp)
            age = self.get_clock().now().nanoseconds * 1.e-9 - stamp
            if not np.isfinite(age) or age < -.005 or stamp_seconds(message.header.stamp) <= 0:
                raise RuntimeError('Invalid or future robot message timestamp')
            if key in self.source_times:
                self.source_times.move_to_end(key)
                return self.source_times[key]
            mapped = min(received, time.monotonic() - max(0., age))
            self.source_times[key] = mapped
            if len(self.source_times) > 256:
                self.source_times.popitem(last=False)
            return mapped

        def sample(self):
            if self.robot is None or self.gripper is None:
                raise RuntimeError('Waiting for real robot and gripper state')
            self.ensure_kinematics()
            robot, received = self.robot
            gripper, gripper_received = self.gripper
            positions = dict(zip(robot.measured_joint_state.name, robot.measured_joint_state.position))
            velocities = dict(zip(robot.measured_joint_state.name, robot.measured_joint_state.velocity))
            q, dq = np.array([positions[n] for n in self.names]), np.array([velocities[n] for n in self.names])
            predicted_tcp, jac = self.kinematics.forward(q, jacobian=True)
            runtime = self.settings
            world_base = np.asarray(runtime['world_T_base'], dtype=float)
            world_ee = world_base @ pose_matrix(robot.o_t_ee.pose)
            tcp = world_ee @ np.asarray(runtime['ee_T_tcp'], dtype=float)
            error = pose_error(predicted_tcp, tcp)
            consistent = (np.linalg.norm(error[:3]) <= runtime['fk_translation_tolerance_m'] and
                          np.linalg.norm(error[3:]) <= runtime['fk_angle_tolerance_rad'])
            world_k = world_ee @ pose_matrix(robot.ee_t_k.pose)
            wrench = robot.o_f_ext_hat_k.wrench
            force = world_base[:3, :3] @ [wrench.force.x, wrench.force.y, wrench.force.z]
            torque = world_base[:3, :3] @ [wrench.torque.x, wrench.torque.y, wrench.torque.z]
            fingers = dict(zip(gripper.name, gripper.position))
            width = sum(fingers[f'left_fr3_finger_joint{i}'] for i in (1, 2))
            twist = jac @ dq
            timestamp = self.message_time(robot, received)
            healthy = robot.robot_mode in (robot.ROBOT_MODE_IDLE, robot.ROBOT_MODE_MOVE) and consistent
            errors = robot.current_errors.get_fields_and_field_types()
            healthy = healthy and not any(getattr(robot.current_errors, k) for k in errors)
            return q, dict(time_s=timestamp, world_T_tcp=tcp,
                tcp_linear_velocity_world=twist[:3], tcp_angular_velocity_world=twist[3:],
                wrench_world_at_k=np.r_[force, torque], world_T_k=world_k, wrench_time_s=timestamp,
                gripper_time_s=self.message_time(gripper, gripper_received), gripper_width_m=width,
                gripper_healthy=bool(np.isfinite(width)), gripper_grasped=None,
                robot_healthy=bool(healthy), wrench_available=bool(consistent),
                force_source='franka_external_estimate')

        def controller_state(self):
            now = time.monotonic()
            if self.controller is None or not 0 <= now - self.controller[1] < self.settings['controller_state_max_age_s']:
                raise RuntimeError('Executor state missing or stale')
            state = self.controller[0]
            stamp = stamp_seconds(state.stamp)
            age = self.get_clock().now().nanoseconds * 1.e-9 - stamp
            if (not np.isfinite(stamp) or stamp <= 0 or not np.isfinite(age) or
                    age < -.005 or age >= self.settings['controller_state_max_age_s']):
                raise RuntimeError('Executor source timestamp invalid, future or stale')
            if (state.state not in ('HOLDING', 'TRACKING', 'STOPPING', 'STOPPED') or
                    not isinstance(state.session, int) or state.session <= 0):
                raise RuntimeError('Invalid executor phase/session')
            return state

        def execution_preflight(self):
            if not self.allow_execution:
                raise RuntimeError('Runtime is read-only; explicit --allow-execution is required')
            self.session.observer.validate_for_execution()
            if self.settings.get('limits_verified') is not True:
                raise RuntimeError('Real runtime limits are unverified')
            params = self.call(self.parameters_client, GetParameters.Request(names=[
                'enabled', 'calibration_verified', 'runtime_limits_verified']))
            if len(params.values) != 3 or not all(v.type == 1 and v.bool_value for v in params.values):
                raise RuntimeError('Executor commissioning gates are not all enabled')
            values = self.call(self.parameters_client, GetParameters.Request(names=[
                'command_timeout_s', 'max_velocity', 'stopped_velocity_rad_s'])).values
            if len(values) != 3 or [v.type for v in values] != [3, 8, 3]:
                raise RuntimeError('Executor timing/velocity limits unavailable')
            timeout, speed, stopped = values[0].double_value, np.asarray(values[1].double_array_value), values[2].double_value
            if (speed.shape != (7,) or not np.isfinite(speed).all() or np.any(speed <= 0) or
                    not np.isfinite([timeout, stopped]).all() or min(timeout, stopped) <= 0 or
                    self.settings['command_valid_for_s'] > timeout or
                    self.session.policy.limits.joint_speed_rad_s > np.min(speed)):
                raise RuntimeError('Runtime command lifetime/speed incompatible with executor limits')
            self.executor_limits = dict(timeout=float(timeout), speed=speed, stopped=float(stopped))

        def reconcile_ownership(self):
            """Inspect a failed/uncertain switch; never replay or reactivate trajectories."""
            try:
                reply = self.call(self.list_client, ListControllers.Request())
                self.controller_inventory = {c.name: c.state for c in reply.controller}
                source = self.settings.get('trajectory_controller', 'left_fr3_arm_controller')
                target = self.settings.get('insertion_controller', 'left_insertion_controller')
                self.owner = self.controller_inventory.get(target) == 'active'
                self.ownership_uncertain = self.owner == (self.controller_inventory.get(source) == 'active')
            except Exception:
                self.ownership_uncertain = True
            # An in-flight switch can still complete after a timeout. No automatic recovery.

        def activate(self, q):
            self.execution_preflight()
            listed = self.call(self.list_client, ListControllers.Request())
            states = {c.name: c.state for c in listed.controller}
            source = self.settings.get('trajectory_controller', 'left_fr3_arm_controller')
            target = self.settings.get('insertion_controller', 'left_insertion_controller')
            if states.get(source) != 'active' or states.get(target) != 'inactive':
                raise RuntimeError('Expected active trajectory controller and inactive insertion controller')
            self.validate_scene()
            # Entry collision check includes the actual whole robot state.
            self.validate_joint_goal(q, q)
            request = SwitchController.Request()
            request.activate_controllers = [target]
            request.deactivate_controllers = [source]
            request.strictness = SwitchController.Request.STRICT
            request.timeout.sec = 2
            self.controller = None
            switch_stamp = self.get_clock().now().nanoseconds * 1.e-9
            self.switch_requested = self.ownership_uncertain = True
            # Monitor HOLDING while the switch RPC is pending: waiting for the RPC first
            # could consume the executor's entire first-command grace period.
            result, done = {}, threading.Event()
            def switch():
                try:
                    result['response'] = self.call(self.switch_client, request, timeout=3.)
                except Exception as exc:
                    result['error'] = exc
                finally:
                    done.set()
            threading.Thread(target=switch, daemon=True).start()
            first_sent = acknowledged = False
            deadline = time.monotonic() + 3.2
            try:
                while time.monotonic() < deadline:
                    if done.is_set():
                        if 'error' in result:
                            raise RuntimeError(str(result['error']))
                        if not result['response'].ok:
                            raise RuntimeError('Strict controller switch failed')
                    try:
                        state = self.controller_state()
                    except RuntimeError:
                        state = None
                    if state is not None and stamp_seconds(state.stamp) >= switch_stamp:
                        if not first_sent and state.state == 'HOLDING' and state.session != self.controller_session:
                            self.controller_session, self.sequence = state.session, 0
                            self.owner, self.ownership_uncertain = True, False
                            self.last_target_stamp = self.last_target_q = None
                            measured_q = np.asarray(state.positions, dtype=float)
                            if measured_q.shape != (7,) or not np.isfinite(measured_q).all():
                                raise RuntimeError('Invalid measured executor joints in handshake')
                            self.q_reference = measured_q.copy()
                            self.publish_target(measured_q)
                            first_sent = True
                        elif first_sent:
                            if state.session != self.controller_session or state.state not in ('HOLDING', 'TRACKING'):
                                raise RuntimeError('Executor stopped during handshake: ' + state.reason)
                            acknowledged = state.state == 'TRACKING' and state.last_sequence >= 1
                            if acknowledged and done.is_set():
                                # Start the policy from the same measured joint pose used for
                                # the first target; never reuse pre-switch IK/reference state.
                                self.session.goal = self.kinematics.forward(self.q_reference)
                                return
                            elapsed = self.get_clock().now().nanoseconds * 1.e-9 - self.last_target_stamp
                            if acknowledged and elapsed >= 1. / self.settings['frequency_hz']:
                                self.publish_target(self.q_reference)
                    time.sleep(.001)
                raise RuntimeError('Executor did not acknowledge the new control session')
            except Exception:
                self.stop_executor()
                self.reconcile_ownership()
                if not done.is_set():
                    self.ownership_uncertain = True
                raise

        def publish_target(self, q):
            state = self.controller_state()
            if state.session != self.controller_session or state.state not in ('HOLDING', 'TRACKING'):
                raise RuntimeError('Executor ownership lost or stopped: ' + state.reason)
            q = np.asarray(q, dtype=float)
            if q.shape != (7,) or not np.isfinite(q).all() or self.executor_limits is None:
                raise RuntimeError('Invalid target or missing executor preflight')
            message = JointTarget()
            message.stamp = self.get_clock().now().to_msg()
            stamp = stamp_seconds(message.stamp)
            if self.last_target_stamp is not None:
                dt = stamp - self.last_target_stamp
                if not 0 < dt <= self.settings['command_valid_for_s']:
                    raise RuntimeError('Command clock gap exceeds verified validity')
                if np.any(np.abs(q - self.last_target_q) > self.executor_limits['speed'] * dt + 1.e-10):
                    raise RuntimeError('Joint target exceeds executor per-stamp velocity bound')
            self.sequence += 1
            message.sequence, message.session = self.sequence, self.controller_session
            message.valid_for_s = self.settings['command_valid_for_s']
            message.positions = [float(v) for v in q]
            message.velocities = [0.] * 7
            self.targets.publish(message)
            self.last_target_stamp, self.last_target_q = stamp, q.copy()

        def stop_executor(self):
            self.running = False
            if (self.owner or self.switch_requested) and not self.stop_requested:
                # Keep the effort controller active: its stopping/holding state
                # owns the arm even if this process exits afterwards.
                self.stop_requested = True
                self.stop_requested_at = time.monotonic()
                try:
                    self.stop_future = self.stop_client.call_async(Trigger.Request())
                except Exception as exc:
                    self.executor_failure = 'Stop request failed: ' + str(exc)

        def monitor_executor(self):
            if not self.owner:
                return None
            try:
                state = self.controller_state()
                if state.session != self.controller_session:
                    raise RuntimeError('Executor ownership session changed')
                velocities = np.asarray(state.velocities, dtype=float)
                if velocities.shape != (7,) or not np.isfinite(velocities).all():
                    raise RuntimeError('Executor measured velocities invalid')
                if state.state in ('STOPPING', 'STOPPED') and not self.stop_requested:
                    raise RuntimeError('Executor stopped unexpectedly: ' + state.reason)
                if self.stop_requested:
                    if self.stop_future is not None and self.stop_future.done():
                        response = self.stop_future.result()
                        self.stop_service_accepted = response is not None and response.success
                        if not self.stop_service_accepted:
                            raise RuntimeError('Executor rejected stop request')
                    self.stop_acknowledged = bool(state.state == 'STOPPED' and state.reason == 'cancelled' and
                        self.executor_limits is not None and
                        np.max(np.abs(velocities)) <= self.executor_limits['stopped'])
                    if state.state in ('STOPPING', 'STOPPED') and state.reason != 'cancelled':
                        raise RuntimeError('Executor fault during stop: ' + state.reason)
                    if not self.stop_acknowledged and time.monotonic() - self.stop_requested_at > 2.:
                        raise RuntimeError('Executor stop not confirmed within two seconds')
                return state
            except Exception as exc:
                self.stop_acknowledged = False
                self.executor_failure = str(exc)
                raise

        def validate_joint_goal(self, start, goal, *, allow_contact=False):
            if self.all_joints is None:
                raise RuntimeError('Missing complete joint state for collision checking')
            self.right_check(ready=True)
            right_positions, _, right_fingers = self.right_sources()
            message, received = self.all_joints
            if time.monotonic() - self.message_time(message, received) > self.settings['controller_state_max_age_s']:
                raise RuntimeError('Full robot collision state stale')
            names = list(message.name)
            if any(n not in names for n in [f'{side}_fr3_joint{i}' for side in ('left', 'right') for i in range(1, 8)]):
                raise RuntimeError('Collision state must contain both arms')
            values = np.array(message.position, dtype=float)
            if values.shape != (len(names),) or not np.isfinite(values).all():
                raise RuntimeError('Invalid complete joint state')
            # The aggregate can keep publishing cached values from a dead right source.
            # Use independently stamped right-arm and gripper observations for the check.
            for name, value in dict(right_positions, **right_fingers).items():
                if name not in names or abs(values[names.index(name)] - value) > .002:
                    raise RuntimeError('Aggregate collision state disagrees with independent right state')
                values[names.index(name)] = value
            count = max(1, int(np.ceil(np.max(np.abs(goal - start)) / .001)))
            for blend in np.linspace(0., 1., count + 1):
                request = GetStateValidity.Request()
                request.robot_state.joint_state.name = names
                joints = values.copy()
                for n, v in zip(self.names, start + blend * (goal - start)):
                    joints[names.index(n)] = v
                request.robot_state.joint_state.position = joints.tolist()
                request.robot_state.is_diff = True
                response = self.call(self.validity_client, request, timeout=self.settings['collision_timeout_s'])
                unexpected = [c for c in response.contacts if {c.contact_body_1, c.contact_body_2} !=
                              {'usb_cable_demo_plug', 'usb_socket'}]
                if not response.valid and (not allow_contact or not response.contacts or unexpected or
                                           response.constraint_result):
                    raise RuntimeError('Local state collision rejected')
            # Reject stale checks if the other arm moved during the request.
            latest, _ = self.all_joints
            current = dict(zip(latest.name, latest.position))
            if any(abs(current.get(n, float('inf')) - v) > .002
                   for n, v in zip(names, values) if n not in self.names):
                raise RuntimeError('Non-inserting joints changed during collision approval')

        def validate_scene(self):
            request = GetPlanningScene.Request(components=PlanningSceneComponents(components=(
                PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
                PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)))
            scene = self.call(self.scene_client, request).scene
            sockets = [obj for obj in scene.world.collision_objects if obj.id == 'usb_socket']
            attachments = [obj for obj in scene.robot_state.attached_collision_objects
                           if obj.object.id == 'usb_cable_demo_plug']
            if len(sockets) != 1 or not (sockets[0].meshes or sockets[0].primitives):
                raise RuntimeError('Collision scene missing socket geometry')
            socket = sockets[0]
            if (socket.header.frame_id != 'world' or len(socket.mesh_poses) != 1 or
                    socket.primitive_poses or not np.allclose(pose_matrix(socket.mesh_poses[0]),
                    self.session.calibration['world_T_socket'], atol=1.e-7, rtol=0.)):
                raise RuntimeError('Socket collision pose differs from calibrated world pose')
            if (len(attachments) != 1 or attachments[0].link_name != 'left_fr3_hand_tcp' or
                    not (attachments[0].object.meshes or attachments[0].object.primitives)):
                raise RuntimeError('Collision scene missing calibrated TCP plug attachment')
            if any(obj.id == 'usb_cable_demo_plug' for obj in scene.world.collision_objects):
                raise RuntimeError('Collision scene contains duplicate world plug')
            plug = attachments[0].object
            if plug.header.frame_id != 'left_fr3_hand_tcp' or len(plug.mesh_poses) != 1 or plug.primitive_poses:
                raise RuntimeError('Plug collision geometry must use one calibrated TCP mesh pose')
            if not np.allclose(pose_matrix(plug.mesh_poses[0]), self.session.calibration['tcp_T_usb'],
                               atol=1.e-7, rtol=0.):
                raise RuntimeError('Collision attachment differs from calibrated grasp')

        def right_sources(self):
            if self.right_robot is None or self.right_gripper is None:
                raise RuntimeError('Independent right-arm or right-gripper state unavailable')
            arm, arm_received = self.right_robot
            gripper, gripper_received = self.right_gripper
            now = time.monotonic()
            if now - self.message_time(arm, arm_received) > self.settings['controller_state_max_age_s']:
                raise RuntimeError('Independent right-arm state stale')
            if now - self.message_time(gripper, gripper_received) > self.session.calibration['max_gripper_age_s']:
                raise RuntimeError('Independent right-gripper state stale')
            positions = dict(zip(arm.name, arm.position))
            speeds = dict(zip(arm.name, arm.velocity))
            fingers = dict(zip(gripper.name, gripper.position))
            expected = [f'right_fr3_joint{i}' for i in range(1, 8)]
            finger_names = [f'right_fr3_finger_joint{i}' for i in (1, 2)]
            if (any(name not in positions or name not in speeds for name in expected) or
                    any(name not in fingers for name in finger_names) or
                    not np.isfinite([positions[n] for n in expected] + [speeds[n] for n in expected] +
                                    [fingers[n] for n in finger_names]).all()):
                raise RuntimeError('Independent right state has invalid joint fields')
            return ({n: positions[n] for n in expected}, {n: speeds[n] for n in expected},
                    {n: fingers[n] for n in finger_names})

        def right_check(self, ready):
            values, speeds, fingers = self.right_sources()
            if min(fingers.values()) < self.settings['right_min_half_width_m']:
                raise RuntimeError('Right gripper has not opened')
            if ready:
                for i, target in enumerate(self.settings['right_ready_q'], 1):
                    if (abs(values.get(f'right_fr3_joint{i}', float('inf')) - target) > .01 or
                            abs(speeds.get(f'right_fr3_joint{i}', float('inf'))) > .02):
                        raise RuntimeError('Right arm must be ready and stationary')

        def snapshot(self):
            snapshot = self.session.snapshot()
            c = self.session.calibration
            executor_state, executor_reason = 'UNKNOWN', self.executor_failure
            if self.owner:
                try:
                    executor = self.monitor_executor()
                    executor_state, executor_reason = executor.state, executor.reason
                except Exception as exc:
                    executor_reason = str(exc)
            # The policy has only reached an estimated target. Pipeline success is withheld
            # until the executor itself reports a fresh, low-speed, cancelled STOPPED hold.
            if snapshot.get('insertion_success') and not (self.stop_acknowledged and not self.executor_failure):
                snapshot['insertion_success'], snapshot['outcome'] = False, None
                snapshot['state'] = 'blocked' if self.executor_failure else 'success_verification'
                snapshot['reason'] = self.executor_failure or 'awaiting_executor_stopped_hold'
            snapshot.update(socket_world_pose=pose_dict(np.asarray(c['world_T_socket'])),
                calibrated_mount=pose_dict(np.asarray(c['tcp_T_usb']), frame='left_fr3_hand_tcp'),
                local_controller_owns_left_arm=self.owner, execution_enabled=self.allow_execution,
                calibration_id=c['calibration_id'], config_fingerprint=config_fingerprint(config),
                executor_state=executor_state, executor_reason=executor_reason,
                stop_requested=self.stop_requested, stop_service_accepted=self.stop_service_accepted,
                stop_acknowledged=self.stop_acknowledged, ownership_uncertain=self.ownership_uncertain,
                controller_inventory=dict(self.controller_inventory), no_auto_reactivate=True)
            return snapshot

        def command(self, operation, response):
            if operation in ('status', 'heartbeat'):
                if operation == 'heartbeat':
                    self.heartbeat = time.monotonic()
                response.success, response.message = True, _json(self.snapshot())
                return response
            if not self.lock.acquire(blocking=False):
                response.success, response.message = False, 'Insertion operation busy'
                return response
            try:
                now = time.monotonic()
                if operation == 'cancel':
                    if self.owner and not self.session.policy.insertion_success:
                        self.session.stop('client_cancel')
                    self.stop_executor()
                elif operation == 'target':
                    if self.owner or self.switch_requested or self.session.policy.state not in ('not_started', 'right_ready', 'approach'):
                        raise RuntimeError('Approach target cannot reset an active or previously attempted insertion')
                    q, sample = self.sample()
                    now = time.monotonic()
                    self.session.observe(now, sample)
                    c = self.session.calibration
                    tcp = sample['world_T_tcp']
                    goal = tcp_goal(np.asarray(c['world_T_socket']), tcp,
                        tcp @ np.asarray(c['tcp_T_usb']), -self.session.policy.limits.preinsert_m,
                        c.get('hole_center_m', [0, .0175, .0686]),
                        tip_in_usb_m=c.get('tip_in_usb_m', [0, .0179, 0]))
                    self.session.policy.transition('approach')
                    response.success, response.message = True, _json(dict(self.snapshot(), target=pose_dict(goal)))
                    return response
                elif operation in ('align', 'start'):
                    q, sample = self.sample()
                    now = time.monotonic()
                    self.right_check(ready=True)
                    if operation == 'align':
                        self.session.begin(now, sample, align=True)
                        self.activate(q)
                        # The switch RPC is outside the control loop. Validate current sources
                        # afresh without interpreting that deliberate consumer pause as lost samples.
                        _, sample = self.sample()
                        self.session.observer.reset()
                        now = time.monotonic()
                        observed = self.session.observe(now, sample)
                        failure = self.session.policy.observation_failure(observed)
                        if failure:
                            raise RuntimeError(failure[1])
                        self.session.last_time = now
                    else:
                        if not self.owner or self.session.policy.state != 'aligned':
                            raise RuntimeError('Start requires completed alignment in this control session')
                        approved_goal = self.session.goal.copy()
                        self.session.begin(now, sample, align=False)
                        self.session.goal = approved_goal
                    self.running, self.heartbeat = True, now
                elif operation == 'right_release':
                    self.execution_preflight()
                    self.right_sources()
                    _, sample = self.sample()
                    now = time.monotonic()
                    obs = self.session.observe(now, sample)
                    if self.session.policy.observation_failure(obs):
                        raise RuntimeError('Invalid insertion observation before preparation')
                    if self.session.policy.state != 'not_started':
                        raise RuntimeError('Right preparation cannot replay')
                    self.session.policy.transition('right_releasing')
                elif operation == 'right_released':
                    if self.session.policy.state != 'right_releasing':
                        raise RuntimeError('Right release acknowledgement out of order')
                    self.right_check(ready=False)
                    self.session.policy.right_gripper_released = True
                    self.session.policy.transition('right_returning')
                elif operation == 'right_returned':
                    if self.session.policy.state != 'right_returning':
                        raise RuntimeError('Right return acknowledgement out of order')
                    self.right_check(ready=True)
                    self.session.policy.right_return_complete = True
                    self.session.policy.transition('right_ready')
                elif operation.startswith('fail_'):
                    self.session.stop(operation)
                    self.stop_executor()
                else:
                    raise RuntimeError('Real first-stage insertion does not provide physical retention or automatic release')
                response.success, response.message = True, _json(self.snapshot())
            except Exception as exc:
                if operation in ('align', 'start'):
                    self.session.stop(str(exc))
                    self.stop_executor()
                response.success, response.message = False, str(exc)
            finally:
                self.lock.release()
            return response

        def tick(self):
            if not self.lock.acquire(blocking=False):
                return
            try:
                if self.owner:
                    self.monitor_executor()
                q, sample = self.sample()
                now = time.monotonic()
                observation = self.session.observe(now, sample)
                if self.owner:
                    failure = self.session.policy.observation_failure(observation)
                    if failure:
                        raise RuntimeError(failure[1])
                if self.owner and self.running:
                    if now - self.heartbeat > self.settings['heartbeat_timeout_s']:
                        raise RuntimeError('Client heartbeat expired')
                    if self.session.policy.state == 'aligned':
                        self.publish_target(self.q_reference)
                    else:
                        goal = self.session.next_goal(now, sample)
                        if goal is None:
                            if self.session.policy.state == 'aligned':
                                self.publish_target(self.q_reference)
                            else:
                                self.stop_executor()
                        else:
                            elapsed = self.get_clock().now().nanoseconds * 1.e-9 - self.last_target_stamp
                            if not 0 < elapsed <= self.settings['command_valid_for_s']:
                                raise RuntimeError('Policy target scheduling exceeded executor validity')
                            max_step = min(self.session.policy.limits.joint_speed_rad_s,
                                float(np.min(self.executor_limits['speed']))) * elapsed
                            target = self.kinematics.solve(goal, self.q_reference, max_step)
                            self.validate_joint_goal(q, target,
                                allow_contact=self.session.policy.state in self.session.policy.ACTIVE)
                            self.publish_target(target)
                            self.q_reference = target
                            self.session.accept_goal(goal)
                snapshot = self.snapshot()
                self.publisher.publish(String(data=_json(snapshot)))
                if self.log:
                    self.log.write(_json(dict(kind='runtime_snapshot', monotonic_time_s=now, **snapshot)) + '\n')
                    self.log.flush()
            except Exception as exc:
                self.session.observation = dict(self.session.observation,
                    observation_valid=False, feedback_available=False, invalid_reason=str(exc))
                if self.owner:
                    self.executor_failure = str(exc)
                    self.stop_acknowledged = False
                    self.session.stop(str(exc))
                    self.stop_executor()
                # Expose executor failures even after policy completion. A stale controller
                # must never leave the last published success as the only available status.
                self.publisher.publish(String(data=_json(self.snapshot())))
                self.get_logger().warning(str(exc), throttle_duration_sec=2.)
            finally:
                self.lock.release()

        def close(self):
            self.stop_executor()
            if self.log:
                self.log.close()

    return RealInsertionNode()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--check-config', action='store_true', help='No ROS initialization or motion')
    parser.add_argument('--allow-execution', action='store_true', help='Allow service-triggered execution after commissioning gates')
    args, ros_args = parser.parse_known_args(argv)
    config = load_real_config(args.config)
    if args.check_config:
        session = RealInsertionSession(config)
        readiness = 'unverified'
        try:
            session.observer.validate_for_execution()
            readiness = 'calibration_verified'
        except ValueError:
            pass
        print(_json(dict(config_valid=True, calibration=readiness,
            limits_verified=config['real_runtime'].get('limits_verified') is True,
            stop_resistance_N=session.policy.limits.speed_m_s / session.policy.limits.resistance_gain_m_N_s)))
        return 0
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=ros_args)
    node = make_node(config, args.allow_execution)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0
