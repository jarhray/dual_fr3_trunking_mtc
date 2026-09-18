"""Backend-neutral stage descriptions and task-to-stage compilation."""

from dual_fr3_trunking_mtc.stages.compiler import build_mtc_stage_specs
from dual_fr3_trunking_mtc.stages.specs import (
    MtcStageSpec,
    mtc_stage_sequence_to_dict,
    mtc_stage_sequence_to_json,
    mtc_stage_sequence_to_text,
    mtc_stage_spec_to_dict,
)

__all__ = [
    "MtcStageSpec",
    "build_mtc_stage_specs",
    "mtc_stage_sequence_to_dict",
    "mtc_stage_sequence_to_json",
    "mtc_stage_sequence_to_text",
    "mtc_stage_spec_to_dict",
]
