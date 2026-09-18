"""Old callers and new responsibility packages must share module globals."""
import importlib

import pytest


@pytest.mark.parametrize('old,new', [
    ('models', 'task.models'), ('planner', 'task.planner'),
    ('scheduler', 'task.scheduler'), ('preparation', 'task.preparation'),
    ('gripper', 'execution.gripper'), ('simulation_cable', 'execution.simulation_cable'),
    ('insertion', 'insertion_task.pipeline'), ('insertion_motion', 'insertion_task.motion'),
    ('insertion_planning', 'insertion_task.planning'),
    ('insertion_planning_scene', 'insertion_task.planning_scene'),
    ('insertion_skill', 'insertion_task.cli'), ('mtc_prototype', 'nodes.prototype'),
    ('ros_node', 'nodes.planner'), ('readiness', 'nodes.readiness'),
    ('segment_executor', 'nodes.segment_executor'), ('markers', 'nodes.markers'),
])
def test_legacy_module_is_canonical_module(old, new):
    assert importlib.import_module('dual_fr3_trunking_mtc.' + old) is importlib.import_module(
        'dual_fr3_trunking_mtc.' + new)
