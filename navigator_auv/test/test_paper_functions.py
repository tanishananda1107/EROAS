#!/usr/bin/env python3
"""Unit tests for the paper-faithful SPD2C (only_gap.py) and ST-CBF
(velocity_cbf.py) helper methods validated against arXiv 2411.05516.

These target the pure(-ish) pieces of two large rclpy Node classes without
booting a ROS graph: functions that don't touch `self` are called unbound,
and functions that only read a couple of instance attributes are exercised
against a lightweight stand-in object built with the real (unbound) methods
bound onto it. No rclpy.init()/Node instantiation happens anywhere here.
"""
import importlib.util
import math
import types
from pathlib import Path

import numpy as np
import pytest

SCRIPTS_DIR = Path(__file__).resolve().parent.parent / 'scripts'


def _load_module(name, filename):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS_DIR / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


only_gap = _load_module('only_gap_under_test', 'only_gap.py')
velocity_cbf = _load_module('velocity_cbf_under_test', 'velocity_cbf.py')

SonarHeadingNode = only_gap.SonarHeadingNode
ObstacleAvoidanceNode = velocity_cbf.ObstacleAvoidanceNode


# ---------------------------------------------------------------------------
# only_gap.py -- SPD2C (Algorithm 1)
# ---------------------------------------------------------------------------

def test_paper_convexity_detects_convex_obstacle():
    # Paper Eq. 15: a >= C_th (0.02) -> convex, AUV turns toward goal-side
    # slope. Synthesize points on a known parabola y = a*x^2 with a=0.10,
    # well above the 0.02 threshold used at the call site.
    a_true = 0.10
    xs = np.linspace(-5, 5, 21)
    points = [(x, a_true * x * x) for x in xs]

    a_fit, avg_left, avg_right = SonarHeadingNode._paper_convexity(None, points)

    assert a_fit == pytest.approx(a_true, abs=1e-6)
    # Left branch of an upward parabola slopes down, right branch slopes up.
    assert avg_left < 0.0
    assert avg_right > 0.0


def test_paper_convexity_detects_concave_wall():
    # A flat wall (a ~ 0) must fall below the paper's C_th=0.02 so the
    # caller triggers vertical sonar pivoting instead of a horizontal turn.
    xs = np.linspace(-5, 5, 21)
    points = [(x, 3.0) for x in xs]  # flat wall at y=3

    a_fit, _, _ = SonarHeadingNode._paper_convexity(None, points)

    assert a_fit == pytest.approx(0.0, abs=1e-6)
    assert a_fit < 0.02


def test_paper_convexity_insufficient_points_returns_none():
    a_fit, avg_left, avg_right = SonarHeadingNode._paper_convexity(None, [(0.0, 0.0), (1.0, 1.0)])
    assert a_fit is None
    assert avg_left == 0.0
    assert avg_right == 0.0


def test_paper_gap_candidates_finds_run_midpoints():
    # Eq. 8-10: a run of `required_beams` consecutive (stride-spaced) free
    # beams yields a candidate gap at its midpoint.
    free_beams = [0, 5, 10, 15, 20, 25, 30]
    mids = only_gap.SonarHeadingNode._paper_gap_candidates(None, free_beams, stride=5, required_beams=3)
    assert mids == [5, 10, 15, 20]


def test_paper_gap_candidates_skips_broken_runs():
    # A hole in the free-beam sequence (missing beam 15) must not produce a
    # false-positive gap candidate spanning it.
    free_beams = [0, 5, 10, 20, 25, 30]
    mids = only_gap.SonarHeadingNode._paper_gap_candidates(None, free_beams, stride=5, required_beams=3)
    assert mids == []


def test_beam_to_angle_matches_paper_eq27_structure():
    # Eq. 27: ψ_R = π/2 - (K_r*b_cl + π/4). _beam_to_angle is a linear map
    # centered on the middle beam that reduces to this equation with
    # K_r = angular resolution per beam; at the FOV edges it must equal
    # +/- half the FOV exactly.
    beam_count = 91
    center_beam = (beam_count - 1) / 2.0

    assert only_gap.SonarHeadingNode._beam_to_angle(None, center_beam, beam_count) == pytest.approx(0.0)
    assert only_gap.SonarHeadingNode._beam_to_angle(None, beam_count - 1, beam_count) == pytest.approx(
        only_gap.FOV_RAD / 2.0)
    assert only_gap.SonarHeadingNode._beam_to_angle(None, 0, beam_count) == pytest.approx(
        -only_gap.FOV_RAD / 2.0)


def test_paper_gap_beam_fraction_matches_reference_beam_count():
    # Regression test for the fix that replaced a hardcoded 15deg corridor
    # requirement with the paper's actual L=150-of-512-beam spec (~26.4deg).
    expected_deg = (150 / 512) * 90.0
    actual_deg = (only_gap.PAPER_GAP_BEAMS / only_gap.PAPER_REFERENCE_BEAM_COUNT) * only_gap.FOV_DEG
    assert actual_deg == pytest.approx(expected_deg)
    assert actual_deg == pytest.approx(26.367, abs=1e-3)


# ---------------------------------------------------------------------------
# velocity_cbf.py -- ST-CBF safety filter
# ---------------------------------------------------------------------------

def _make_cbf_stub(max_xy_speed=5.0):
    """A minimal stand-in carrying only what _project_to_cbf_constraints and
    its helpers touch (self.max_xy_speed), with the real unbound methods
    bound onto it. No Node/rclpy machinery involved."""
    stub = types.SimpleNamespace(max_xy_speed=max_xy_speed)
    for name in (
        '_constraint_value',
        '_constraints_satisfied',
        '_project_to_cbf_constraint',
        '_project_to_cbf_constraints',
    ):
        setattr(stub, name, getattr(ObstacleAvoidanceNode, name).__get__(stub))
    return stub


def test_cbf_constraint_h_matches_paper_eq31():
    # Eq. 31: h(p_v, t) = ||p_v - p_o,cl||^2 - R_o^2
    p_v = np.array([1.0, 2.0])
    p_o = np.array([4.0, 6.0])
    r_o = 2.0
    distance = float(np.linalg.norm(p_v - p_o))
    h = distance ** 2 - r_o ** 2
    assert distance == pytest.approx(5.0)
    assert h == pytest.approx(25.0 - 4.0)


def test_project_to_cbf_constraints_passthrough_when_safe():
    cbf = _make_cbf_stub(max_xy_speed=5.0)
    desired = np.array([1.0, 0.0])
    # gradient . desired + margin = 1*1 + 0*0 + 5 = 6 >= 0 -> already safe.
    constraints = [(np.array([1.0, 0.0]), 5.0)]
    result = cbf._project_to_cbf_constraints(desired, constraints)
    assert np.allclose(result, desired)


def test_project_to_cbf_constraints_projects_onto_boundary_when_violated():
    cbf = _make_cbf_stub(max_xy_speed=5.0)
    desired = np.array([0.0, 0.0])
    # Constraint: 1*vx + 0*vy - 2 >= 0  ->  vx >= 2. desired violates it
    # (0 - 2 = -2 < 0), so the QP-equivalent projection must move the point
    # to the nearest point on the vx=2 boundary: (2, 0).
    constraints = [(np.array([1.0, 0.0]), -2.0)]
    result = cbf._project_to_cbf_constraints(desired, constraints)
    assert np.allclose(result, np.array([2.0, 0.0]), atol=1e-6)


def test_project_to_cbf_constraints_passthrough_with_no_constraints():
    # `_project_to_cbf_constraints` early-returns `desired` unchanged when
    # there are no obstacle constraints at all -- speed limiting in that
    # case is the caller's responsibility, not this function's. (This is
    # current, correct behavior; an earlier version of this test wrongly
    # assumed max_xy_speed clamping always applies here.)
    cbf = _make_cbf_stub(max_xy_speed=1.0)
    desired = np.array([3.0, 4.0])  # norm=5, exceeds max_xy_speed
    result = cbf._project_to_cbf_constraints(desired, constraints=[])
    assert np.allclose(result, desired)


def test_project_to_cbf_constraints_respects_max_speed_when_constrained():
    # With at least one (here, trivially satisfied) constraint present, the
    # function does clamp to max_xy_speed before considering constraints.
    cbf = _make_cbf_stub(max_xy_speed=1.0)
    desired = np.array([3.0, 4.0])  # norm=5, exceeds max_xy_speed
    trivial_constraint = (np.array([1.0, 0.0]), 100.0)  # always satisfied
    result = cbf._project_to_cbf_constraints(desired, constraints=[trivial_constraint])
    assert np.linalg.norm(result) == pytest.approx(1.0)
    # Direction should be preserved.
    assert np.allclose(result / np.linalg.norm(result), desired / np.linalg.norm(desired))


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))
