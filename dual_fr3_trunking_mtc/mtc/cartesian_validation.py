"""Reject Cartesian trajectories that leave their intended positional path."""

import copy
import logging
import math

import numpy as np

from dual_fr3_trunking_mtc.mtc.path_length import LENGTH_EPSILON, sampled_tcp_positions
from dual_fr3_trunking_mtc.mtc.diagnostics import append_failure_comment


LOGGER = logging.getLogger(__name__)


def validate_cartesian_settings(jump_threshold: float, path_tolerance: float):
    if not math.isfinite(jump_threshold) or jump_threshold <= 1.0:
        raise ValueError("cartesian_jump_threshold must be finite and > 1.0")
    if not math.isfinite(path_tolerance) or path_tolerance <= 0.0:
        raise ValueError("cartesian_path_tolerance must be finite and > 0.0")


def cartesian_path_cost(link_name, tolerance, stage_name, *, stationary=False):
    """
    Check the time-parameterized candidate before MTC propagates it.

    MoveIt checks Cartesian IK samples, but joint interpolation and timing can
    move the TCP away from those samples. Check dense FK positions against the
    finite start/end segment, or the initial TCP position for an in-place turn.
    Full endpoint completion remains the Cartesian planner's responsibility.
    This numerical check does not model a controller's spline or tracking error.
    """
    if not math.isfinite(tolerance) or tolerance <= 0.0:
        raise ValueError("cartesian_path_tolerance must be finite and > 0.0")

    def cost(solution, _comment=None):
        try:
            trajectory = solution.trajectory
            if trajectory is None or len(trajectory) == 0:
                raise ValueError("Cartesian trajectory is empty")
            if not solution.start.scene.knows_frame_transform(trajectory[0], link_name):
                raise ValueError(f"unknown TCP frame {link_name!r}")
            positions = sampled_tcp_positions(trajectory, link_name)
            start = next(positions)
            last = copy.copy(trajectory[len(trajectory) - 1])
            last.update()
            end = start if stationary else last.get_global_link_transform(link_name)[:3, 3]
            if not np.isfinite(end).all():
                raise ValueError("non-finite TCP endpoint")
            direction = end - start
            squared_distance = float(np.dot(direction, direction))
            max_deviation = 0.0
            length = 0.0
            previous = start
            for position in positions:
                fraction = (
                    float(np.clip(
                        np.dot(position - start, direction) / squared_distance, 0.0, 1.0,
                    ))
                    if squared_distance > 0.0 else 0.0
                )
                closest = start + fraction * direction
                max_deviation = max(max_deviation, float(np.linalg.norm(position - closest)))
                length += float(np.linalg.norm(position - previous))
                previous = position
            accepted = max_deviation <= tolerance + LENGTH_EPSILON
            LOGGER.log(
                logging.INFO if accepted else logging.ERROR,
                "%s: Cartesian TCP %s deviation %.6f m; limit %.6f m: %s",
                stage_name, "in-place" if stationary else "line", max_deviation,
                tolerance, "accepted" if accepted else "REJECTED",
            )
            if not accepted:
                append_failure_comment(solution, (
                    f"CARTESIAN_PATH_DEVIATION: {stage_name} TCP {link_name} "
                    f"{'in-place' if stationary else 'line'} deviation {max_deviation:.6f} m "
                    f"> cartesian_path_tolerance={tolerance:.6f} m"
                ))
            return length if accepted else math.inf
        except Exception as exc:  # noqa: BLE001 - never propagate an unchecked path
            append_failure_comment(solution, f"CARTESIAN_CHECK_ERROR: {stage_name}: {exc}")
            LOGGER.exception("%s: Cartesian TCP check failed: %s", stage_name, exc)
            return math.inf

    return cost


def merged_cartesian_path_cost(link_names, tolerance, stage_name):
    """Recheck each TCP after Merger resampling and time parameterization."""
    checks = tuple(
        cartesian_path_cost(link, tolerance, f"{stage_name}/{link}")
        for link in link_names
    )

    def cost(solution, _comment=None):
        return sum(check(solution) for check in checks)

    return cost
