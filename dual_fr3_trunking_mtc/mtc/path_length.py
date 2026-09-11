"""TCP path-length acceptance checks for FR3 anchor trajectories."""

from __future__ import annotations

import copy
import logging
import math

import numpy as np

from .diagnostics import append_failure_comment


LOGGER = logging.getLogger(__name__)
# FR3 arm joints are bounded revolute joints. Subdivide their actual commanded
# joint displacements, not just the straight chords between sparse TCP poses.
MAX_JOINT_SAMPLE_STEP = 0.005  # rad
LENGTH_EPSILON = 1e-6  # m, numerical comparison tolerance only


def validate_path_length_ratio(ratio: float) -> float:
    if not math.isfinite(ratio) or ratio < 1.0:
        raise ValueError("anchor_max_path_length_ratio must be finite and >= 1.0")
    return ratio


def sampled_tcp_positions(trajectory, link_name: str):
    """
    Yield dense FK positions without changing the trajectory or its states.

    Joint interpolation follows the bounded FR3 joints' commanded positions.
    This uses linear joint interpolation, independent of trajectory timing.
    """
    if trajectory is None or len(trajectory) == 0:
        raise ValueError("trajectory is empty")

    state = copy.copy(trajectory[0])
    state.update()
    position = state.get_global_link_transform(link_name)[:3, 3].copy()
    if not np.isfinite(position).all():
        raise ValueError("non-finite TCP position")
    start_joints = state.joint_positions
    if not all(math.isfinite(value) for value in start_joints.values()):
        raise ValueError("non-finite trajectory joint position")
    yield position
    for index in range(1, len(trajectory)):
        end_joints = trajectory[index].joint_positions
        if not all(math.isfinite(value) for value in end_joints.values()):
            raise ValueError("non-finite trajectory joint position")
        delta = {name: end_joints[name] - value for name, value in start_joints.items()}
        steps = max(1, math.ceil(max(map(abs, delta.values())) / MAX_JOINT_SAMPLE_STEP))
        for step in range(1, steps + 1):
            fraction = step / steps
            state.joint_positions = {
                name: value + fraction * delta[name]
                for name, value in start_joints.items()
            }
            state.update()
            position = state.get_global_link_transform(link_name)[:3, 3].copy()
            if not np.isfinite(position).all():
                raise ValueError("non-finite TCP position")
            yield position
        start_joints = end_joints


def tcp_path_length(trajectory, link_name: str) -> float:
    """Estimate TCP arc length, including motion between sparse waypoints."""
    positions = sampled_tcp_positions(trajectory, link_name)
    previous = next(positions)
    length = 0.0
    for position in positions:
        length += float(np.linalg.norm(position - previous))
        previous = position
    return length


def anchor_path_length_cost(link_name: str, target, ratio: float, stage_name: str):
    """
    Return an MTC cost callback that rejects overlong anchor solutions.

    An infinite MTC cost marks the solution as failed, so it cannot propagate
    into either a whole-task solution or a task replanned for staged execution.
    The baseline uses the actual trajectory start and the requested target,
    expressed in the model frame. It never uses the nominal from-keypoint.
    """
    validate_path_length_ratio(ratio)

    # MTC bindings expose both one- and two-argument cost callback overloads.
    def cost(solution, _comment=None):
        try:
            trajectory = solution.trajectory
            if trajectory is None or len(trajectory) == 0:
                raise ValueError("anchor trajectory is empty")
            scene = solution.start.scene
            first_state = trajectory[0]
            for frame in (link_name, target.header.frame_id):
                if not scene.knows_frame_transform(first_state, frame):
                    raise ValueError(f"unknown frame {frame!r}")
            first_state = copy.copy(first_state)
            first_state.update()
            start = first_state.get_global_link_transform(link_name)[:3, 3]
            point = target.pose.position
            target_in_model = scene.get_frame_transform(target.header.frame_id) @ np.array(
                [point.x, point.y, point.z, 1.0]
            )
            distance = float(np.linalg.norm(target_in_model[:3] - start))
            if not math.isfinite(distance):
                raise ValueError("non-finite start-to-target distance")
            limit = ratio * distance
            length = tcp_path_length(trajectory, link_name)
            accepted = length <= limit + LENGTH_EPSILON
            message = (
                f"TCP path {length:.6f} m; limit {limit:.6f} m "
                f"({ratio:g} x start-to-target {distance:.6f} m): "
                f"{'accepted' if accepted else 'REJECTED'}"
            )
            LOGGER.log(
                logging.INFO if accepted else logging.ERROR,
                "%s: %s", stage_name, message,
            )
            if not accepted:
                append_failure_comment(solution, f"PATH_LENGTH_LIMIT: {message}")
            return length if accepted else math.inf
        except Exception as exc:  # noqa: BLE001 - never accept an unchecked path
            append_failure_comment(solution, f"PATH_LENGTH_CHECK_ERROR: {exc}")
            LOGGER.exception("%s: TCP path length check failed: %s", stage_name, exc)
            return math.inf

    return cost
