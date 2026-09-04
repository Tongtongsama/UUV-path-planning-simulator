import numpy as np
import pytest

pytest.importorskip("scipy")

from validation.compare_fossen_model import run_validation


def test_rk4_converges_against_solve_ivp_with_expected_trend():
    results = run_validation()
    for case in results["cases"]:
        euler = case["methods"]["euler"]
        rk4 = case["methods"]["rk4"]
        assert all(np.diff(euler["l2_final_errors"]) < 0.0)
        assert all(np.diff(rk4["l2_final_errors"]) < 0.0)
        assert euler["observed_orders"][-1] > 0.8
        assert rk4["observed_orders"][-1] > 3.5
        assert rk4["l2_final_errors"][-1] < euler["l2_final_errors"][-1]
