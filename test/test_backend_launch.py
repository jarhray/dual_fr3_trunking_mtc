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
def test_launch_defaults_to_gazebo_with_one_backend_selector(package, filename):
    description = load_launch(package, filename)
    context = LaunchContext()
    apply_arguments(description, context)
    assert context.launch_configurations["simulation_backend"] == "gazebo"
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
