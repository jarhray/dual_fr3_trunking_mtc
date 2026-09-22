"""Bounded local IK on the actual URDF chain; no ROS service in the servo loop."""
from dataclasses import dataclass
import xml.etree.ElementTree as ET

import numpy as np
from transforms3d.axangles import axangle2mat
from transforms3d.euler import euler2mat
from transforms3d.quaternions import mat2quat


def _vector(text, default):
    value = np.asarray([float(x) for x in text.split()] if text else default, dtype=float)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError('URDF vector must have three finite components')
    return value


def pose_error(current, goal):
    quat = mat2quat(goal[:3, :3] @ current[:3, :3].T)
    if quat[0] < 0:
        quat = -quat
    sine = np.linalg.norm(quat[1:])
    rotation = 2 * quat[1:] if sine < 1.e-10 else quat[1:] * (2 * np.arctan2(sine, quat[0]) / sine)
    return np.r_[goal[:3, 3] - current[:3, 3], rotation]


@dataclass
class Joint:
    name: str
    kind: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


class LocalKinematics:
    def __init__(self, description, root='world', tip='left_fr3_hand_tcp', joint_names=None):
        robot = ET.fromstring(description)
        by_child = {j.find('child').get('link'): j for j in robot.findall('joint')}
        chain, seen, link = [], set(), tip
        while link != root:
            if link in seen or link not in by_child:
                raise ValueError(f'No unique URDF chain from {root} to {tip}')
            seen.add(link)
            element = by_child[link]
            kind = element.get('type')
            if kind not in ('fixed', 'revolute', 'prismatic') or element.find('mimic') is not None:
                raise ValueError('Local IK only supports fixed and bounded independent joints')
            origin = np.eye(4)
            raw_origin = element.find('origin')
            if raw_origin is not None:
                origin[:3, 3] = _vector(raw_origin.get('xyz'), [0, 0, 0])
                origin[:3, :3] = euler2mat(*_vector(raw_origin.get('rpy'), [0, 0, 0]))
            axis_node = element.find('axis')
            axis = _vector(None if axis_node is None else axis_node.get('xyz'), [1, 0, 0])
            norm = np.linalg.norm(axis)
            if norm < 1.e-12:
                raise ValueError('Invalid joint axis')
            bounds = element.find('limit')
            if kind != 'fixed' and (bounds is None or bounds.get('lower') is None or bounds.get('upper') is None):
                raise ValueError('Bounded URDF joints require explicit lower and upper limits')
            lo, hi = (0., 0.) if kind == 'fixed' else (
                float(bounds.get('lower')), float(bounds.get('upper')))
            if not np.isfinite([lo, hi]).all() or (kind != 'fixed' and lo >= hi):
                raise ValueError('Invalid URDF joint limits')
            chain.append(Joint(element.get('name'), kind, origin, axis / norm, lo, hi))
            link = element.find('parent').get('link')
        self.chain = list(reversed(chain))
        self.names = [j.name for j in self.chain if j.kind != 'fixed']
        if len(self.names) != 7 or (joint_names is not None and self.names != list(joint_names)):
            raise ValueError('Expected the configured seven arm joints in URDF chain order')
        self.lower = np.array([j.lower for j in self.chain if j.kind != 'fixed'])
        self.upper = np.array([j.upper for j in self.chain if j.kind != 'fixed'])

    def forward(self, q, jacobian=False):
        q = np.asarray(q, dtype=float)
        if q.shape != (7,) or not np.isfinite(q).all():
            raise ValueError('Expected seven finite joint positions')
        transform, index, axes = np.eye(4), 0, []
        for joint in self.chain:
            transform = transform @ joint.origin
            if joint.kind == 'fixed':
                continue
            axes.append((joint.kind, transform[:3, :3] @ joint.axis, transform[:3, 3].copy()))
            motion = np.eye(4)
            if joint.kind == 'revolute':
                motion[:3, :3] = axangle2mat(joint.axis, q[index])
            else:
                motion[:3, 3] = joint.axis * q[index]
            transform = transform @ motion
            index += 1
        if not jacobian:
            return transform
        result = np.zeros((6, 7))
        for i, (kind, axis, point) in enumerate(axes):
            if kind == 'revolute':
                result[:3, i] = np.cross(axis, transform[:3, 3] - point)
                result[3:, i] = axis
            else:
                result[:3, i] = axis
        return transform, result

    def solve(self, goal, seed, max_joint_step, *, iterations=16,
              position_tolerance=1.e-5, angle_tolerance=1.e-4):
        q = np.array(seed, dtype=float, copy=True)
        origin = q.copy()
        if (q.shape != (7,) or not np.isfinite(q).all() or
                np.any(q < self.lower) or np.any(q > self.upper) or
                not np.isfinite(max_joint_step) or max_joint_step <= 0):
            raise ValueError('Invalid bounded IK seed or step')
        goal = np.asarray(goal, dtype=float)
        if (goal.shape != (4, 4) or not np.isfinite(goal).all() or
                not np.allclose(goal[3], [0, 0, 0, 1], atol=1.e-8, rtol=0.) or
                not np.allclose(goal[:3, :3].T @ goal[:3, :3], np.eye(3), atol=1.e-6, rtol=0.) or
                not np.isclose(np.linalg.det(goal[:3, :3]), 1., atol=1.e-6, rtol=0.)):
            raise ValueError('Invalid IK target')
        low, high = np.maximum(self.lower, origin - max_joint_step), np.minimum(self.upper, origin + max_joint_step)
        for _ in range(iterations):
            current, jac = self.forward(q, jacobian=True)
            error = pose_error(current, goal)
            if (np.linalg.norm(error[:3]) <= position_tolerance and
                    np.linalg.norm(error[3:]) <= angle_tolerance):
                return q
            # Damping bounds near-singular amplification; the outer envelope
            # guarantees no branch jump even if the target is unreachable.
            delta = jac.T @ np.linalg.solve(jac @ jac.T + np.eye(6) * 1.e-8, error)
            q = np.clip(q + delta, low, high)
        raise RuntimeError('local_IK_failed_within_joint_step_envelope')
