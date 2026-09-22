"""Engine/ROS independent insertion session shared by replay and the real node."""
import copy
import hashlib
import json

import numpy as np

from dual_fr3_maniskill.usb.alignment import correction_goal
from dual_fr3_maniskill.usb.insertion import InsertionPolicy, ALIGNMENT_STATES
from dual_fr3_maniskill.usb.observation import CalibratedObservation, _transform


def config_fingerprint(config):
    return hashlib.sha256(json.dumps(config, sort_keys=True, allow_nan=False).encode()).hexdigest()


def load_real_config(path):
    import yaml
    with open(path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or not isinstance(config.get('insertion'), dict):
        raise ValueError('Expected an insertion mapping')
    config = copy.deepcopy(config)
    insertion = config['insertion']
    if insertion.get('observation_mode') != 'calibrated_estimate':
        raise ValueError('Real insertion requires observation_mode: calibrated_estimate')
    if any(insertion.get(key, True) is not False for key in (
            'retain_after_success', 'release_after_retention', 'return_after_release')):
        raise ValueError('Real first-stage insertion must keep the plug held, without simulated retention')
    CalibratedObservation(insertion['calibration'])
    InsertionPolicy(insertion)
    runtime = config.get('real_runtime', {})
    for key in ('frequency_hz', 'max_dt_s', 'command_valid_for_s', 'controller_state_max_age_s',
                'fk_translation_tolerance_m', 'fk_angle_tolerance_rad', 'heartbeat_timeout_s',
                'collision_timeout_s', 'right_min_half_width_m'):
        value = runtime.get(key)
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not np.isfinite(value) or value <= 0:
            raise ValueError(f'real_runtime.{key} must be positive finite')
    if runtime['max_dt_s'] < 1. / runtime['frequency_hz']:
        raise ValueError('max_dt_s must allow the configured policy period')
    for name in ('world_T_base', 'ee_T_tcp'):
        _transform(runtime.get(name), name)
    ready = np.asarray(runtime.get('right_ready_q'), dtype=float)
    if ready.shape != (7,) or not np.isfinite(ready).all():
        raise ValueError('right_ready_q must contain seven finite angles')
    if not isinstance(runtime.get('limits_verified'), bool):
        raise ValueError('limits_verified must be explicitly true or false')
    if runtime['right_min_half_width_m'] >= .04:
        raise ValueError('Right opening must fit the gripper travel')
    if runtime['command_valid_for_s'] < 1. / runtime['frequency_hz']:
        raise ValueError('Command validity shorter than the policy period')
    if runtime.get('local_collision_check') != 'per_step':
        raise ValueError('Real local execution currently requires per_step collision checking')
    return config


class RealInsertionSession:
    """Position increments only. Motion ownership and collision approval are external."""
    def __init__(self, config):
        self.config = config
        self.policy = InsertionPolicy(config['insertion'])
        self.observer = CalibratedObservation(config['insertion']['calibration'])
        self.calibration = config['insertion']['calibration']
        self.goal = None
        self.last_time = None
        self.observation = {}
        self.record_policy_event = None

    def record(self, now, operation, **details):
        if self.record_policy_event is not None:
            self.record_policy_event(dict(kind='policy_event', time_s=now,
                operation=operation, observation=self.observation, **details))

    def observe(self, now, sample):
        self.observation = self.observer.observe(now, sample)
        return self.observation

    def begin(self, now, sample, *, align):
        self.observer.validate_for_execution()
        if self.config['real_runtime'].get('limits_verified') is not True:
            raise RuntimeError('Real contact and execution limits have not been verified')
        observation = self.observe(now, sample)
        if align:
            self.policy.begin_alignment(now, observation)
        else:
            self.policy.begin(now, observation)
        self.record(now, 'begin_alignment' if align else 'begin')
        self.goal = np.array(sample['world_T_tcp'], dtype=float, copy=True)
        self.last_time = now

    def stop(self, reason):
        if not self.policy.insertion_success:
            self.policy.stop('blocked', reason)

    def next_goal(self, now, sample):
        observation = self.observe(now, sample)
        if self.last_time is None:
            raise RuntimeError('Session has not started')
        dt = float(now - self.last_time)
        if not 0 < dt <= self.config['real_runtime']['max_dt_s']:
            self.stop('policy_clock_or_update_gap')
            return None
        self.last_time = now
        tcp = np.asarray(sample['world_T_tcp'])
        if np.linalg.norm(self.goal[:3, 3] - tcp[:3, 3]) > self.policy.limits.tracking_limit_m:
            self.stop('joint_drive_tracking_error')
            return None
        if self.policy.state in ALIGNMENT_STATES:
            correcting = self.policy.update_alignment(now, observation)
            self.record(now, 'alignment_update')
            if not correcting:
                return self.goal.copy() if self.policy.state in ALIGNMENT_STATES else None
            base = np.asarray(self.calibration['world_T_socket'])
            usb = tcp @ np.asarray(self.calibration['tcp_T_usb'])
            target = correction_goal(base, tcp, usb,
                self.calibration.get('hole_center_m', [0, .0175, .0686]),
                self.policy.limits.preinsert_m, self.policy.alignment_limits, dt,
                tip_in_usb_m=self.calibration.get('tip_in_usb_m', [0, .0179, 0]))
            # Same measured correction integrated on the held target as simulation.
            goal = target @ np.linalg.inv(tcp) @ self.goal
            from dual_fr3_maniskill.usb.alignment import rotation_distance
            distance = float(np.linalg.norm(goal[:3, 3] - self.goal[:3, 3]))
            angle = float(rotation_distance(goal[:3, :3], self.goal[:3, :3]))
            self.policy.record_alignment_step(distance, angle)
            self.record(now, 'alignment_step', distance_m=distance, angle_rad=angle)
        elif self.policy.state in self.policy.ACTIVE:
            speed = self.policy.update(now, observation)
            self.record(now, 'update')
            if self.policy.state not in self.policy.ACTIVE:
                return None
            goal = self.goal.copy()
            goal[:3, 3] -= np.asarray(self.calibration['world_T_socket'])[:3, 0] * speed * dt
        else:
            return None
        return goal

    def accept_goal(self, goal):
        """Commit only after IK and collision validation approve the command."""
        self.goal = np.array(goal, copy=True)

    def snapshot(self):
        result = self.policy.snapshot()
        result['observation'] = self.observation
        result['backend'] = 'real'
        result['physical_success_verified'] = False
        return result
