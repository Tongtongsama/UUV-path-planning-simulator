"""Reuse the frozen integration/PID loop and its independent raw replay."""
from unittest.mock import patch
from .config import CASES,parameters,flow_context
from ros2.twin_axial_g12_matrix import runner
from ros2.twin_axial_s3_transient_v2 import cli

def run(output,case,source_sha):
 with flow_context(case),patch.object(runner,'load_synthetic_parameters',lambda:parameters(case)):
  return runner.run_case(output,'S2',case,'P0',source_sha)

def verify(output,case):
 with flow_context(case),patch.object(cli,'load_synthetic_parameters',lambda:parameters(case)):
  return cli.verify_run(output,replay=True)
