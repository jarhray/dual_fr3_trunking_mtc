"""Real counterpart of the insertion client; never creates physical objects."""
import json
from pathlib import Path

import numpy as np

from .real_session import load_real_config, config_fingerprint


class RealInsertionClient:
    def __init__(self, config_path):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from std_srvs.srv import Trigger
        from moveit_msgs.srv import ApplyPlanningScene
        self.config = load_real_config(config_path)
        self.config_path = str(Path(config_path).resolve())
        self.service_prefix = self.config['real_runtime'].get('service_prefix', '/real/usb/insertion/')
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = Node('real_insertion_client', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.apply = self.node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.status = self.node.create_client(Trigger, self.service_prefix + 'status')

    def _call(self, client, request, *, require_success=True):
        import rclpy
        if not client.wait_for_service(timeout_sec=3.):
            raise RuntimeError('Unavailable service: ' + client.srv_name)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, executor=self.executor, timeout_sec=10.)
        if not future.done():
            raise RuntimeError('Service timeout; no retry: ' + client.srv_name)
        response = future.result()
        if response is None or (require_success and not getattr(response, 'success', False)):
            raise RuntimeError(getattr(response, 'message', 'Service failed'))
        return response

    def checked_status(self):
        from std_srvs.srv import Trigger
        from dual_fr3_maniskill.usb.insertion import InsertionPolicy
        snapshot = json.loads(self._call(self.status, Trigger.Request()).message)
        if snapshot.get('config_fingerprint') != config_fingerprint(self.config):
            raise RuntimeError('Runtime and client configurations differ')
        if InsertionPolicy(self.config['insertion']).observation_failure(snapshot.get('observation', {})):
            raise RuntimeError('Invalid calibrated grasp or real feedback')
        return snapshot

    def sync_grasp(self, _config_path, orientation_direction='forward'):
        from ament_index_python.packages import get_package_share_directory
        from dual_fr3_maniskill.cable.planning_scene import attached_usb_scene
        from dual_fr3_maniskill.usb.geometry import pose_dict
        from dual_fr3_trunking_mtc.execution.simulation_cable import _observed_pose
        from moveit_msgs.srv import ApplyPlanningScene
        self.checked_status()
        source = self.config.get('geometry_config')
        if not source:
            source = str(Path(get_package_share_directory('dual_fr3_maniskill')) /
                         'config/trunking_cable_simplified_2mm.yaml')
        elif not Path(source).is_absolute():
            source = str(Path(self.config_path).parent / source)
        scene = attached_usb_scene(source, orientation_direction=orientation_direction)
        obj = scene.robot_state.attached_collision_objects[0].object
        mount = np.asarray(self.config['insertion']['calibration']['tcp_T_usb'])
        obj.mesh_poses = [_observed_pose({'pose': pose_dict(mount)}, 'pose')]
        self._call(self.apply, ApplyPlanningScene.Request(scene=scene))
        return True

    def ensure_grasp(self):
        self.checked_status()
        return True

    def close(self):
        self.node.destroy_node()
        self.executor.shutdown()
        self.context.shutdown()
