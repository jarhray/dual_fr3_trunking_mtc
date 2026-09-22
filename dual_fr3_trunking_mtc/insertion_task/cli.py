"""Independent ROS entry for the existing terminal insertion skill.

Connects to an already running MoveIt/ManiSkill scene with a released stable
USB grasp. No scene launch, cable spawn, grasp operation or Enter prompt here.
"""
import argparse
import sys

from rclpy.utilities import remove_ros_args

from dual_fr3_trunking_mtc.execution.gripper import GripperController, GripperProfileRegistry
from dual_fr3_trunking_mtc.insertion_task.pipeline import TerminalInsertion, estimated_hold_valid
from dual_fr3_trunking_mtc.mtc.task_builder import import_mtc_modules
from dual_fr3_trunking_mtc.runtime.arguments import _default_gripper_profiles_file
from dual_fr3_trunking_mtc.runtime.config import parse_bool
from dual_fr3_trunking_mtc.runtime.console_logging import make_logger
from dual_fr3_trunking_mtc.execution.simulation_cable import SimulationCableController


def run_skill(terminal, gripper, *, execute):
    status = terminal.command('status')
    if status['return_complete'] or estimated_hold_valid(status):
        return True
    if status['state'] != 'not_started':
        raise RuntimeError('Insertion skill already attempted; reset before replay')
    if not terminal.plan_from_current_state():
        return False
    return terminal.execute(gripper) if execute else True


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cable-config', default='')
    parser.add_argument('--gripper-profiles-file', default=_default_gripper_profiles_file())
    parser.add_argument('--execute', type=parse_bool, default=None)
    parser.add_argument('--backend', choices=('maniskill', 'real'), default='maniskill')
    args = parser.parse_args(remove_ros_args(args=[parser.prog, *(sys.argv[1:] if argv is None else argv)])[1:])
    if args.execute is None:
        args.execute = args.backend == 'maniskill'
    from dual_fr3_maniskill.scenes import resolve_cable_config

    if args.backend == 'real' and not args.cable_config:
        raise ValueError('Real insertion requires an explicit calibrated configuration')
    config = args.cable_config if args.backend == 'real' else resolve_cable_config(args.cable_config, scene='trunking_cable')
    logger = make_logger()
    rclcpp, _, _ = import_mtc_modules()
    client = gripper = None
    rclcpp.init()
    try:
        node = rclcpp.Node('usb_insertion_skill', rclcpp.NodeOptions(
            automatically_declare_parameters_from_overrides=True))
        real_config = None
        if args.backend == 'real':
            from dual_fr3_trunking_mtc.insertion_task.real_backend import RealInsertionClient
            client = RealInsertionClient(config)
            real_config = client.config['insertion']
            if args.execute:
                status = client.checked_status()
                if not status.get('execution_enabled'):
                    raise RuntimeError('Real runtime is read-only')
        else:
            client = SimulationCableController(backend='maniskill')
        profiles = GripperProfileRegistry.load(args.gripper_profiles_file)
        if args.execute:
            gripper = GripperController(profiles, 'franka' if args.backend == 'real' else 'maniskill',
                                        use_sim_time=args.backend != 'real')
        terminal = TerminalInsertion(client, config, node, logger, config=real_config)
        return 0 if run_skill(terminal, gripper, execute=args.execute) else 4
    except Exception:
        logger.exception('independent insertion skill stopped')
        return 4
    finally:
        if gripper is not None:
            gripper.close()
        if client is not None:
            client.close()
        rclcpp.shutdown()
