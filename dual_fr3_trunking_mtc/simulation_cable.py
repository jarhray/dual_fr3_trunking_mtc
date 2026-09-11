"""MTC's runtime boundary to the deferred ManiSkill scene."""
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from std_srvs.srv import Trigger


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
        self.apply = self.node.create_client(ApplyPlanningScene, "/apply_planning_scene")

    def _call(self, client, request):
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError(f"Service unavailable: {client.srv_name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, executor=self.executor,
                                         timeout_sec=self.timeout)
        if not future.done():
            # Do not retry: a timed-out insertion may still finish in the simulator.
            raise RuntimeError(f"Service timed out; activation state is uncertain: {client.srv_name}")
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(getattr(response, "message", "Planning scene update failed"))
        return response

    def execute(self, spec):
        from dual_fr3_maniskill.cable.planning_scene import attached_usb_scene
        scene = attached_usb_scene(spec.cable_config)
        # Discover both endpoints before making the physical insertion.
        if not self.apply.wait_for_service(timeout_sec=5.):
            raise RuntimeError("/apply_planning_scene unavailable; cable has not been inserted")
        response = self._call(self.spawn, Trigger.Request())
        self.node.get_logger().info(response.message)
        # Match the attachment already present in the selected full MTC plan.
        self._call(self.apply, ApplyPlanningScene.Request(scene=scene))
        return True

    def close(self):
        self.node.destroy_node()
        self.executor.shutdown()
        self.context.shutdown()
