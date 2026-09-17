"""Terminal operation after successful cached MTC execution (simulation only)."""
import json
import struct
import time
from pathlib import Path
import numpy as np
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.msg import (PlanningScene, CollisionObject, AttachedCollisionObject,
                             AllowedCollisionEntry, PlanningSceneComponents)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import Mesh, MeshTriangle
from std_srvs.srv import Trigger
from .simulation_cable import _observed_pose


class TerminalInsertion:
    def __init__(self, client, config_path, node, logger):
        from dual_fr3_maniskill.cable.model import load_geometry_config
        self.client, self.node, self.logger = client, node, logger
        self.config_path = config_path
        self.config = load_geometry_config(config_path).get('insertion', {})
        self.services = {name: client.node.create_client(Trigger, '/maniskill/usb/insertion/'+name)
            for name in ('target', 'start', 'status', 'heartbeat', 'cancel', 'retained', 'released', 'returned', 'returning', 'fail_release', 'fail_return', 'right_release', 'right_released', 'right_returned', 'fail_right_release', 'fail_right_return')}
        self.get_scene = client.node.create_client(GetPlanningScene, '/get_planning_scene')
        self.saved_acm = None

    def command(self, operation):
        response = self.client._call(self.services[operation], Trigger.Request())
        return json.loads(response.message)

    def apply(self, scene):
        self.client._call(self.client.apply, ApplyPlanningScene.Request(scene=scene))

    def sync_socket(self):
        from dual_fr3_maniskill.insertion_geometry import HOLE, SOCKET_NAME, SOCKET_INSTALLATION_FIXTURES
        status = self.command('status')
        pose = _observed_pose(status, 'socket_world_pose')
        path = Path(get_package_share_directory('dual_fr3_maniskill'))/self.config.get('collision_mesh','meshes/usb_base_collision.stl')
        raw = path.read_bytes()
        mesh = Mesh()
        hole = np.asarray(self.config.get('hole_center_m', HOLE), dtype=float)
        half = np.array([.00445,.012])/2+np.array(self.config.get('clearance_yz_m',[.0004,.0004]))
        for offset in range(84, len(raw), 50):
            values = struct.unpack_from('<12fH', raw, offset)
            start = len(mesh.vertices)
            for index in (3,6,9):
                x,y,z=values[index:index+3]
                if .0624599 <= z <= .0746601 and .0149999 <= y <= .0199001:
                    y=hole[1]+(-half[0] if y<HOLE[1] else half[0])
                    z=hole[2]+(-half[1] if z<HOLE[2] else half[1])
                mesh.vertices.append(Point(x=float(x),y=float(y),z=float(z)))
            mesh.triangles.append(MeshTriangle(vertex_indices=[start,start+1,start+2]))
        obj=CollisionObject(id=SOCKET_NAME,operation=CollisionObject.ADD,meshes=[mesh],mesh_poses=[pose])
        obj.header.frame_id='world'
        scene=PlanningScene(is_diff=True)
        scene.world.collision_objects=[obj]
        response = self.client._call(self.get_scene, GetPlanningScene.Request(
            components=PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)),
            require_success=False)
        acm = response.scene.allowed_collision_matrix
        for name in (SOCKET_NAME, *SOCKET_INSTALLATION_FIXTURES):
            if name not in acm.entry_names:
                acm.entry_names.append(name)
                for row in acm.entry_values:
                    row.enabled.append(False)
                acm.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(acm.entry_names)))
        index = acm.entry_names.index(SOCKET_NAME)
        for name in SOCKET_INSTALLATION_FIXTURES:
            other = acm.entry_names.index(name)
            acm.entry_values[index].enabled[other] = True
            acm.entry_values[other].enabled[index] = True
        scene.allowed_collision_matrix = acm
        self.apply(scene)

    def motion(self, name, group, target, *, cartesian=False, link=None, execute=True):
        from .mtc.task_builder import import_mtc_modules, create_motion_planners, _identity_ik_frame
        from .mtc.path_length import terminal_path_length_cost
        _, core, stages = import_mtc_modules()
        attempts = int(self.config.get('planning_attempts', 10))
        for attempt in range(1, attempts+1):
            # A failed plan has issued no motion. Reconstruct CurrentState and
            # the pipeline so each OMPL search can obtain a fresh sample set.
            task = core.Task()
            task.name = name
            task.loadRobotModel(self.node)
            task.add(stages.CurrentState('measured_state'))
            if not execute:
                expected = stages.ModifyPlanningScene('expected_USB_socket_contact_only')
                expected.allowCollisions('usb_cable_demo_plug', ['usb_socket'], True)
                task.add(expected)
            cart, _, ompl = create_motion_planners(core, self.node, .001, .08, .08, 2.)
            move = stages.MoveTo(name, cart if cartesian else ompl)
            move.group = group
            if link:
                move.ik_frame = _identity_ik_frame(link)
            move.setGoal(target)
            move.setCostTerm(terminal_path_length_cost(
                link or group.removesuffix('_arm')+'_hand_tcp', target,
                float(self.config.get('max_path_length_ratio', 1.5)), name, group=group))
            move.timeout = float(self.config.get('planning_timeout_s', 15.))
            task.add(move)
            self.logger.info('%s planning attempt %d/%d', name, attempt, attempts)
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
        self.logger.info('terminal insertion motion completed: %s', name)

    def detach_measured_usb(self, status):
        from dual_fr3_maniskill.cable.model import USB_LINK
        from dual_fr3_maniskill.cable.planning_scene import usb_collision_object
        from dual_fr3_maniskill.cable.threading import TOUCH_LINKS
        scene=PlanningScene(is_diff=True); scene.robot_state.is_diff=True
        scene.robot_state.attached_collision_objects=[AttachedCollisionObject(
            object=CollisionObject(id=USB_LINK,operation=CollisionObject.REMOVE))]
        obj=usb_collision_object(self.config_path)
        obj.header.frame_id='world'
        obj.mesh_poses=[_observed_pose(status['observation'],'usb_world_pose')]
        scene.world.collision_objects=[obj]
        # Keep only the pre-existing grasp contact pairs permitted while fingers
        # withdraw. The simulator's collision groups are never changed.
        response=self.client._call(self.get_scene,GetPlanningScene.Request(components=PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)),require_success=False)
        acm=response.scene.allowed_collision_matrix
        import copy
        self.saved_acm=copy.deepcopy(acm)
        for name in [USB_LINK,*TOUCH_LINKS]:
            if name not in acm.entry_names:
                acm.entry_names.append(name)
                for row in acm.entry_values: row.enabled.append(False)
                acm.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(acm.entry_names)))
        usb=acm.entry_names.index(USB_LINK)
        for name in TOUCH_LINKS:
            index=acm.entry_names.index(name)
            acm.entry_values[usb].enabled[index]=True
            acm.entry_values[index].enabled[usb]=True
        scene.allowed_collision_matrix=acm
        self.apply(scene)

    def withdraw(self, side):
        from moveit.task_constructor import core, stages
        from geometry_msgs.msg import Vector3Stamped, Vector3
        from std_msgs.msg import Header
        from .mtc.task_builder import create_motion_planners, _identity_ik_frame
        from .mtc.path_length import terminal_path_length_cost
        attempts = int(self.config.get('planning_attempts', 10))
        for attempt in range(attempts):
            task = core.Task(); task.name = side+'_socket_withdraw'; task.loadRobotModel(self.node)
            task.add(stages.CurrentState('measured_open_state'))
            cart, _, _ = create_motion_planners(core, self.node, .001, .05, .05, 2.)
            move = stages.MoveRelative('withdraw_open_'+side, cart)
            move.group = side+'_fr3_arm'; link = side+'_fr3_hand_tcp'
            move.ik_frame = _identity_ik_frame(link)
            direction = Vector3Stamped(header=Header(frame_id=link),
                vector=Vector3(z=-float(self.config.get('retreat_m', .05))))
            move.setDirection(direction)
            move.setCostTerm(terminal_path_length_cost(link, direction,
                float(self.config.get('max_path_length_ratio', 1.5)), side+'_socket_withdraw'))
            task.add(move)
            if task.plan(1) and task.solutions:
                break
        else:
            raise RuntimeError(side+' withdrawal planning failed')
        if not task.execute(task.solutions[0]):
            raise RuntimeError(side+' withdrawal execution failed')

    def execute(self, gripper_controller):
        from .gripper import GripperRequest
        retained=False
        released=False
        phase = 'initial'
        try:
            status=self.command('status')
            if status['return_complete']:
                return True
            if status['state']!='not_started':
                raise RuntimeError('Terminal operation already attempted; reset before replay')
            phase = 'right_release'
            self.command('right_release')
            width = float(self.config.get('release_width_m', {}).get('right', .03))
            if not gripper_controller.execute(GripperRequest(actor='follower', profile='open', width_override=width)):
                raise RuntimeError('right gripper release failed')
            self.command('right_released')
            phase = 'right_return'
            self.withdraw('right')
            self.motion('right_return_ready', 'right_fr3_arm', 'ready')
            self.command('right_returned')
            phase = 'insertion'
            target=self.command('target')
            goal=PoseStamped(); goal.header.frame_id='world'
            goal.pose=_observed_pose({'pose':target},'pose')
            import copy
            lift = float(self.config.get('approach_lift_m', 0.))
            if lift:
                high = copy.deepcopy(goal)
                high.pose.position.z += lift
                self.motion('socket_above_approach','left_fr3_arm',high,link='left_fr3_hand_tcp')
            self.motion('socket_preinsert','left_fr3_arm',goal,cartesian=bool(lift),link='left_fr3_hand_tcp')
            # Correct for the measured grasp after transporting and settling.
            target=self.command('target')
            goal.pose=_observed_pose({'pose':target},'pose')
            self.motion('socket_align_actual_usb','left_fr3_arm',goal,cartesian=True,link='left_fr3_hand_tcp')
            target = self.command('target')
            end = PoseStamped(); end.header.frame_id = 'world'
            end.pose = _observed_pose(target, 'end_tcp_pose')
            self.motion('insertion_collision_preflight','left_fr3_arm',end,
                        cartesian=True,link='left_fr3_hand_tcp',execute=False)
            self.command('start')
            wall=time.monotonic()
            while True:
                status=self.command('heartbeat')  # explicit controller lease, read-only status does not renew it
                if status['state'] not in ('aligned','feedback_advance','success_verification'):
                    break
                if time.monotonic()-wall>max(120.,30*self.config.get('timeout_s',45.)):
                    raise RuntimeError('Terminal insertion wall-time watchdog')
                time.sleep(.1)
            self.logger.info('insertion result: %s',json.dumps(status))
            if not status['insertion_success']:
                raise RuntimeError('Insertion failed: '+status['state']+' '+status['reason'])
            if not self.config.get('retain_after_success',True):
                return True
            status=self.command('retained'); retained=True
            if not self.config.get('release_after_retention',True):
                return True
            self.detach_measured_usb(status)
            width = float(self.config.get('release_width_m', {}).get('left', .08))
            if not gripper_controller.execute(GripperRequest(actor='leader', profile='open', width_override=width)):
                raise RuntimeError('left gripper release failed')
            self.command('released')
            released=True
            if not self.config.get('return_after_release',True):
                return True
            self.command('returning')
            self.withdraw('left')
            if self.saved_acm is not None:
                self.apply(PlanningScene(is_diff=True,allowed_collision_matrix=self.saved_acm))
                self.saved_acm=None
            self.motion('left_return_ready', 'left_fr3_arm', 'ready')
            self.command('returned')
            return True
        except Exception as exc:
            self.logger.error('terminal insertion stopped: %s',exc)
            if phase in ('right_release', 'right_return'):
                try: self.command('fail_'+phase)
                except Exception: self.logger.exception('Unable to report right-arm preparation failure')
            elif retained:
                try: self.command('fail_return' if released else 'fail_release')
                except Exception: self.logger.exception('Unable to report return failure')
            return False
        finally:
            try: self.command('cancel')
            except Exception: self.logger.exception('Unable to acknowledge controller release')
            if self.saved_acm is not None:
                try: self.apply(PlanningScene(is_diff=True,allowed_collision_matrix=self.saved_acm))
                except Exception: self.logger.exception('Unable to restore grasp collision pairs')
