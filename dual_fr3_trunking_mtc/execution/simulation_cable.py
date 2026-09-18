"""MTC's runtime boundary to the deferred ManiSkill scene."""
import json
import math
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParametersAtomically
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


def _observed_pose(snapshot, key):
    """Validate simulation observations before changing planning geometry."""
    from geometry_msgs.msg import Pose
    try:
        raw = snapshot[key]
        position = list(map(float, raw["position_m"]))
        quaternion = list(map(float, raw["quaternion_wxyz"]))
        if len(position) != 3 or len(quaternion) != 4:
            raise ValueError("pose must contain three positions and four quaternion components")
        if not all(math.isfinite(value) for value in position + quaternion):
            raise ValueError("pose must be finite")
        norm = math.sqrt(sum(value*value for value in quaternion))
        if norm < 1.e-12:
            raise ValueError("quaternion must be nonzero")
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = (
            value/norm for value in quaternion)
        return pose
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f"Invalid measured USB {key}: {exc}") from exc


def preparation_cable_config(backend, enabled, preparation_enabled, config_path):
    if backend != "maniskill" or not enabled:
        return ""
    if not preparation_enabled:
        raise ValueError("ManiSkill cable requires preparation_enabled=true; use maniskill_cable=false to run without cable")
    from dual_fr3_maniskill.scenes import resolve_cable_config
    return resolve_cable_config(config_path, scene="trunking_cable")


class SimulationCableController:
    def __init__(self, *, backend, timeout=120.):
        if backend != "maniskill":
            raise ValueError("Simulation cable activation is only supported by ManiSkill")
        self.timeout = timeout
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = Node("trunking_simulation_cable", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.spawn = self.node.create_client(Trigger, "/maniskill/cable/spawn")
        self.prepare = self.node.create_client(SetParametersAtomically, "/dual_fr3_maniskill/set_parameters_atomically")
        self.release = self.node.create_client(Trigger, "/maniskill/usb/release")
        self.verify = self.node.create_client(Trigger, "/maniskill/usb/verify")
        self.status = self.node.create_client(Trigger, "/maniskill/usb/status")
        self.apply = self.node.create_client(ApplyPlanningScene, "/apply_planning_scene")

    def _call(self, client, request, *, require_success=True):
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError(f"Service unavailable: {client.srv_name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, executor=self.executor,
                                         timeout_sec=self.timeout)
        if not future.done():
            # Do not retry: a timed-out insertion may still finish in the simulator.
            raise RuntimeError(f"Service timed out; activation state is uncertain: {client.srv_name}")
        response = future.result()
        success = (response.result.successful if isinstance(response, SetParametersAtomically.Response)
                   else getattr(response, "success", False))
        if response is None or (require_success and not success):
            reason = (response.result.reason if isinstance(response, SetParametersAtomically.Response)
                      else getattr(response, "message", "service returned no successful response"))
            raise RuntimeError(f"{client.srv_name}: {reason}")
        return response

    def execute(self, spec):
        from dual_fr3_maniskill.cable.planning_scene import usb_collision_object
        from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
        from dual_fr3_maniskill.cable.model import USB_LINK
        if not self.apply.wait_for_service(timeout_sec=5.):
            raise RuntimeError("/apply_planning_scene unavailable; physical operation not started")
        if spec.cable_operation == "spawn":
            if spec.cable_preparation_poses:
                payload = json.dumps(spec.cable_preparation_poses, allow_nan=False)
                self._call(self.prepare, SetParametersAtomically.Request(parameters=[Parameter(
                    name="usb_preparation_poses", value=ParameterValue(
                        type=ParameterType.PARAMETER_STRING, string_value=payload))]))
            response = self._call(self.spawn, Trigger.Request())
            self.node.get_logger().info(response.message)
            # Supported USB intentionally reports success=false: inspect the
            # observation, never reconstruct a stationary fixture from a TCP
            # that may have moved since the first idempotent spawn.
            snapshot = json.loads(self._call(self.status, Trigger.Request(), require_success=False).message)
            observed_world = snapshot.get("world_pose") if isinstance(snapshot, dict) else None
            if not isinstance(observed_world, dict) or observed_world.get("frame") != "world":
                raise RuntimeError("USB status lacks a measured world pose; planning scene unchanged")
            pose = _observed_pose(snapshot, "world_pose")
            scene = PlanningScene(is_diff=True)
            scene.robot_state.is_diff = True
            # A previous debug reset removed the physical object, including any
            # world support. Remove a stale planning attachment before re-adding.
            scene.robot_state.attached_collision_objects = [AttachedCollisionObject(
                object=CollisionObject(id=USB_LINK, operation=CollisionObject.REMOVE))]
            obj = usb_collision_object(spec.cable_config,
                orientation_direction=spec.cable_orientation_direction)
            obj.header.frame_id = "world"
            obj.mesh_poses = [pose]
            scene.world.collision_objects = [obj]
        elif spec.cable_operation == "release_verify":
            for client in (self.release, self.verify):
                response = self._call(client, Trigger.Request())
                self.node.get_logger().info(response.message)
            return self.sync_grasp(spec.cable_config, spec.cable_orientation_direction)
        else:
            raise ValueError(f"Unknown simulation USB operation: {spec.cable_operation}")
        self._call(self.apply, ApplyPlanningScene.Request(scene=scene))
        return True

    def sync_grasp(self, config_path, orientation_direction='forward'):
        """Update the planning attachment from an already released, stable grasp.

        Also used by the independent insertion skill; never creates a physical
        support, respawns the object or closes a gripper.
        """
        from dual_fr3_maniskill.cable.planning_scene import attached_usb_scene

        scene = attached_usb_scene(config_path, orientation_direction=orientation_direction)
        snapshot = json.loads(self._call(self.status, Trigger.Request()).message)
        obj = scene.robot_state.attached_collision_objects[0].object
        if (not isinstance(snapshot, dict) or snapshot.get('state') != 'stable' or
                snapshot.get('external_support') is not False or
                snapshot.get('relative_frame') != obj.header.frame_id):
            raise RuntimeError('USB status no longer confirms a released stable grasp; refusing attachment')
        obj.mesh_poses = [_observed_pose(snapshot, 'relative_pose')]
        self._call(self.apply, ApplyPlanningScene.Request(scene=scene))
        return True

    def ensure_grasp(self):
        """Fail before subsequent transport if released-grasp monitoring is not stable."""
        self._call(self.status, Trigger.Request())
        return True

    def close(self):
        self.node.destroy_node()
        self.executor.shutdown()
        self.context.shutdown()
