import numpy as np
import pytest

from four_bar_leg_mpc.geometry import (
    DEFAULT_LEG_PARAMETERS,
    foot_position,
    leg_points,
    passive_joint_angles,
)
from four_bar_leg_mpc.safety import safety_margins


def test_passive_joint_angles_are_finite_across_representative_range() -> None:
    for q_extend in np.linspace(0.6, 5.5, 20):
        passive = passive_joint_angles(float(q_extend))
        assert np.isfinite(passive.fourth_joint)
        assert np.isfinite(passive.fifth_joint)
        assert 0.0 <= passive.gamma <= np.pi


def test_leg_points_and_foot_position_are_consistent() -> None:
    q = np.array([0.1, 3.1, 3.3])
    points = leg_points(q)
    np.testing.assert_allclose(points.foot, foot_position(q))
    assert points.foot.shape == (3,)
    assert np.all(np.isfinite(points.foot))


def test_invalid_joint_shape_is_rejected() -> None:
    with pytest.raises(ValueError, match="shape"):
        leg_points(np.zeros(2))


def test_nominal_configuration_is_inside_demo_joint_limits() -> None:
    q = np.array([0.0, 3.1, 3.3])
    assert np.all(q > DEFAULT_LEG_PARAMETERS.q_min)
    assert np.all(q < DEFAULT_LEG_PARAMETERS.q_max)
    margins = safety_margins(q)
    assert min(margins.values()) > 0.0
