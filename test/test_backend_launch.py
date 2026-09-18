"""Verify the public launch defaults and mutually exclusive backend branches."""
import importlib.util
import logging
from pathlib import Path

import pytest

# ROS launch installs a global logger class whose loggers do not propagate.
# Restore pytest's logger class before other task test modules are imported.
_logger_class = logging.getLoggerClass()
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
logging.setLoggerClass(_logger_class)


SOURCE = Path(__file__).resolve().parents[2]


def load_launch(package, filename):
    spec = importlib.util.spec_from_file_location(
        "backend_launch", SOURCE / package / "launch" / filename,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def apply_arguments(description, context):
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)


@pytest.mark.parametrize("package,filename", [
    ("dual_fr3_moveit_config", "demo.launch.py"),
    ("dual_fr3_trunking_mtc", "demo.launch.py"),
    ("dual_fr3_trunking_mtc", "mtc_prototype.launch.py"),
])
def test_launch_defaults_with_one_backend_selector(package, filename):
    description = load_launch(package, filename)
    context = LaunchContext()
    apply_arguments(description, context)
    expected = 'maniskill' if filename == 'mtc_prototype.launch.py' else 'gazebo'
    assert context.launch_configurations['simulation_backend'] == expected
    if filename == 'mtc_prototype.launch.py':
        assert context.launch_configurations['cable_solver'] == 'rope_actor'
        for name in ('load_cable', 'execute', 'insertion_enabled', 'run_insertion'):
            assert context.launch_configurations[name] == 'true'
    assert "use_gazebo" not in context.launch_configurations
    assert "use_fake_hardware" not in context.launch_configurations
    context.launch_configurations["simulation_backend"] = "auto"
    with pytest.raises(RuntimeError, match="is not valid"):
        apply_arguments(description, context)


@pytest.mark.parametrize("backend", ["gazebo", "maniskill", "fake", "real"])
def test_moveit_launch_selects_exactly_one_backend(backend):
    description = load_launch("dual_fr3_moveit_config", "demo.launch.py")
    context = LaunchContext()
    context.launch_configurations["simulation_backend"] = backend
    apply_arguments(description, context)
    selected = [
        action for action in description.entities
        if isinstance(action, (GroupAction, IncludeLaunchDescription))
        and action.condition.evaluate(context)
    ]
    assert len(selected) == 1
    if backend in ("fake", "real"):
        assert isinstance(selected[0], GroupAction)
    else:
        assert isinstance(selected[0], IncludeLaunchDescription)
        source = selected[0].launch_description_source
        source.get_launch_description(context)
        assert source.location.endswith(f"/{backend}.launch.py")


@pytest.mark.parametrize("backend,enabled,expected", [
    ("maniskill", "true", "trunking_cable"), ("maniskill", "false", "robot"),
    ("maniskill", "1", "trunking_cable"), ("fake", "true", "robot"),
    ("real", "true", "robot"), ("gazebo", "true", "robot"),
])
def test_mtc_entry_selects_deferred_cable_only_for_maniskill(backend, enabled, expected):
    from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
    description = load_launch("dual_fr3_trunking_mtc", "mtc_prototype.launch.py")
    context = LaunchContext()
    context.launch_configurations.update(simulation_backend=backend, maniskill_cable=enabled)
    apply_arguments(description, context)
    include, = [a for a in description.entities if isinstance(a, IncludeLaunchDescription)]
    arguments = dict(include.launch_arguments)
    selected = perform_substitutions(context, normalize_to_list_of_substitutions(arguments["maniskill_scene"]))
    assert selected == expected


@pytest.mark.parametrize("solver", ["mpm", "rope_actor"])
def test_mtc_forwards_cable_solver_to_moveit(solver):
    from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
    description = load_launch("dual_fr3_trunking_mtc", "mtc_prototype.launch.py")
    context = LaunchContext()
    context.launch_configurations.update(simulation_backend="maniskill", cable_solver=solver,
                                         cable_trace_dir="/tmp/rope trace")
    apply_arguments(description, context)
    include, = [a for a in description.entities if isinstance(a, IncludeLaunchDescription)]
    value = dict(include.launch_arguments)["cable_solver"]
    assert perform_substitutions(context, normalize_to_list_of_substitutions(value)) == solver
    trace = dict(include.launch_arguments)["cable_trace_dir"]
    assert perform_substitutions(context, normalize_to_list_of_substitutions(trace)) == "/tmp/rope trace"


@pytest.mark.parametrize("enabled", ["true", "false"])
def test_mtc_forwards_load_cable_and_keeps_usb_scene(enabled):
    from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
    description = load_launch("dual_fr3_trunking_mtc", "mtc_prototype.launch.py")
    context = LaunchContext()
    context.launch_configurations.update(simulation_backend="maniskill", load_cable=enabled)
    apply_arguments(description, context)
    include, = [a for a in description.entities if isinstance(a, IncludeLaunchDescription)]
    arguments = dict(include.launch_arguments)
    value = perform_substitutions(context, normalize_to_list_of_substitutions(arguments["load_cable"]))
    assert value == enabled
    scene = perform_substitutions(context, normalize_to_list_of_substitutions(arguments["maniskill_scene"]))
    assert scene == "trunking_cable"


@pytest.mark.parametrize("direction", ["forward", "reverse"])
def test_mtc_forwards_leader_orientation_to_maniskill(direction):
    from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
    description = load_launch("dual_fr3_trunking_mtc", "mtc_prototype.launch.py")
    context = LaunchContext()
    context.launch_configurations.update(simulation_backend="maniskill",
                                         leader_orientation_direction=direction)
    apply_arguments(description, context)
    include, = [a for a in description.entities if isinstance(a, IncludeLaunchDescription)]
    value = dict(include.launch_arguments)["leader_orientation_direction"]
    assert perform_substitutions(context, normalize_to_list_of_substitutions(value)) == direction


@pytest.mark.parametrize('backend,enabled,expected', [
    ('maniskill', 'true', 'true'), ('maniskill', 'false', 'false'),
    ('gazebo', 'true', 'false'), ('real', 'true', 'false'), ('fake', 'true', 'false'),
])
def test_insertion_default_respects_backend_and_explicit_override(backend, enabled, expected):
    description = load_launch('dual_fr3_trunking_mtc', 'mtc_prototype.launch.py')
    context = LaunchContext()
    context.launch_configurations.update(simulation_backend=backend, maniskill_cable=enabled)
    apply_arguments(description, context)
    assert context.launch_configurations['insertion_enabled'] == expected
    context.launch_configurations['insertion_enabled'] = 'false'
    apply_arguments(description, context)
    assert context.launch_configurations['insertion_enabled'] == 'false'


def test_standalone_launch_contains_only_skill_node():
    from launch.actions import OpaqueFunction
    from launch_ros.actions import Node

    description = load_launch('dual_fr3_trunking_mtc', 'insertion_skill.launch.py')
    context = LaunchContext()
    apply_arguments(description, context)
    factory, = [item for item in description.entities if isinstance(item, OpaqueFunction)]
    node, = factory.execute(context)
    assert isinstance(node, Node)
    assert not any(isinstance(item, IncludeLaunchDescription) for item in description.entities)


def test_mtc_defaults_match_recommended_explicit_command():
    import os
    description = load_launch('dual_fr3_trunking_mtc', 'mtc_prototype.launch.py')
    implicit, explicit = LaunchContext(), LaunchContext()
    # Respect the documented MANISKILL_PYTHON override when present.
    python = os.environ.get('MANISKILL_PYTHON', str(Path.cwd() / '.venv/bin/python'))
    explicit.launch_configurations.update(
        simulation_backend='maniskill', cable_solver='rope_actor',
        load_cable='true', execute='true', maniskill_python=python)
    apply_arguments(description, implicit)
    apply_arguments(description, explicit)
    assert implicit.launch_configurations == explicit.launch_configurations
