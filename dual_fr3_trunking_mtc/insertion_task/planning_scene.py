"""MoveIt message construction for socket installation and measured USB detach.

The caller owns service calls and the lifetime of saved collision permissions.
CAD calculations live in ManiSkill's engine-independent insertion_geometry.
"""
import struct
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from moveit_msgs.msg import (
    AllowedCollisionEntry, AttachedCollisionObject, CollisionObject, PlanningScene,
)
from shape_msgs.msg import Mesh, MeshTriangle
from dual_fr3_maniskill.usb.geometry import (
    DEFAULT_CLEARANCE_YZ_M,
    HOLE,
    SOCKET_INSTALLATION_FIXTURES,
    SOCKET_NAME,
    adjust_hole_vertices,
)

from dual_fr3_trunking_mtc.execution.simulation_cable import _observed_pose


def allow_pairs(acm, body, others):
    """Add symmetric pairs without changing existing entries or defaults."""
    for name in (body, *others):
        if name not in acm.entry_names:
            acm.entry_names.append(name)
            for row in acm.entry_values:
                row.enabled.append(False)
            acm.entry_values.append(
                AllowedCollisionEntry(enabled=[False] * len(acm.entry_names)))
    index = acm.entry_names.index(body)
    for name in others:
        other = acm.entry_names.index(name)
        acm.entry_values[index].enabled[other] = True
        acm.entry_values[other].enabled[index] = True


def socket_collision_object(config, status):
    """Keep the original binary STL face order; only adjust hole-wall vertices."""
    path = Path(get_package_share_directory('dual_fr3_maniskill')) / config.get(
        'collision_mesh', 'meshes/usb_base_collision.stl')
    raw = path.read_bytes()
    vertices = []
    for offset in range(84, len(raw), 50):
        values = struct.unpack_from('<12fH', raw, offset)
        vertices.extend(values[index:index + 3] for index in (3, 6, 9))
    if not config.get('preserve_mesh_geometry', False):
        vertices = adjust_hole_vertices(
            vertices, config.get('clearance_yz_m', DEFAULT_CLEARANCE_YZ_M),
            config.get('hole_center_m', HOLE))
    mesh = Mesh(
        vertices=[Point(x=float(x), y=float(y), z=float(z)) for x, y, z in vertices],
        triangles=[MeshTriangle(vertex_indices=[i, i + 1, i + 2])
                   for i in range(0, len(vertices), 3)],
    )
    obj = CollisionObject(
        id=SOCKET_NAME, operation=CollisionObject.ADD, meshes=[mesh],
        mesh_poses=[_observed_pose(status, 'socket_world_pose')])
    obj.header.frame_id = 'world'
    return obj


def socket_scene(socket, acm):
    allow_pairs(acm, SOCKET_NAME, SOCKET_INSTALLATION_FIXTURES)
    scene = PlanningScene(is_diff=True, allowed_collision_matrix=acm)
    scene.world.collision_objects = [socket]
    return scene


def detached_usb_scene(config_path, status):
    from dual_fr3_maniskill.cable.model import USB_LINK
    from dual_fr3_maniskill.cable.planning_scene import usb_collision_object

    scene = PlanningScene(is_diff=True)
    scene.robot_state.is_diff = True
    scene.robot_state.attached_collision_objects = [AttachedCollisionObject(
        object=CollisionObject(id=USB_LINK, operation=CollisionObject.REMOVE))]
    obj = usb_collision_object(config_path)
    obj.header.frame_id = 'world'
    obj.mesh_poses = [_observed_pose(status['observation'], 'usb_world_pose')]
    scene.world.collision_objects = [obj]
    return scene


def allow_grasp_contacts(scene, acm):
    from dual_fr3_maniskill.cable.model import USB_LINK
    from dual_fr3_maniskill.cable.threading import TOUCH_LINKS

    # Permit the existing grasp contacts only while the fingers withdraw.
    allow_pairs(acm, USB_LINK, TOUCH_LINKS)
    scene.allowed_collision_matrix = acm
