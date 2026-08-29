from __future__ import annotations

from typing import Sequence

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from .models import Keypoint, SegmentPlan


def _color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    return ColorRGBA(r=r, g=g, b=b, a=a)


def _keypoint_color(keypoint: Keypoint) -> ColorRGBA:
    return _color(0.2, 0.8, 0.3) if keypoint.in_slot else _color(0.95, 0.55, 0.1)


def _segment_color(segment: SegmentPlan) -> ColorRGBA:
    if segment.action == "seat_edge":
        return _color(0.9, 0.2, 0.2)
    return _color(0.2, 0.45, 0.95)


def build_marker_array(
    keypoints: Sequence[Keypoint],
    segments: Sequence[SegmentPlan],
    keypoint_scale: float = 0.05,
    segment_width: float = 0.015,
    z_offset: float = 0.0,
    show_labels: bool = True,
) -> MarkerArray:
    marker_array = MarkerArray()

    for idx, keypoint in enumerate(keypoints):
        marker = Marker()
        marker.header.frame_id = keypoint.frame_id
        marker.ns = "keypoints"
        marker.id = idx
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = keypoint.position[0]
        marker.pose.position.y = keypoint.position[1]
        marker.pose.position.z = keypoint.position[2] + z_offset
        marker.pose.orientation.w = 1.0
        marker.scale.x = keypoint_scale
        marker.scale.y = keypoint_scale
        marker.scale.z = keypoint_scale
        marker.color = _keypoint_color(keypoint)
        marker_array.markers.append(marker)

        if show_labels:
            label = Marker()
            label.header.frame_id = keypoint.frame_id
            label.ns = "keypoint_labels"
            label.id = idx
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = keypoint.position[0]
            label.pose.position.y = keypoint.position[1]
            label.pose.position.z = keypoint.position[2] + z_offset + keypoint_scale
            label.pose.orientation.w = 1.0
            label.scale.z = keypoint_scale * 0.55
            label.color = _color(1.0, 1.0, 1.0)
            label.text = keypoint.name
            marker_array.markers.append(label)

    offset = len(keypoints)
    for idx, segment in enumerate(segments):
        marker = Marker()
        marker.header.frame_id = segment.goal.frame_id
        marker.ns = "segments"
        marker.id = offset + idx
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = segment_width
        marker.color = _segment_color(segment)
        for waypoint in segment.waypoints:
            point = Point()
            point.x = waypoint.pose.position.x
            point.y = waypoint.pose.position.y
            point.z = waypoint.pose.position.z + z_offset
            marker.points.append(point)
        marker_array.markers.append(marker)

    return marker_array
