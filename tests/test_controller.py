import numpy as np

from four_bar_leg_mpc.controller import MPCParameters, rollout, solve_mpc
from four_bar_leg_mpc.geometry import foot_position


def test_rollout_integrates_joint_velocity_controls() -> None:
    parameters = MPCParameters(dt=0.1, horizon=2)
    q0 = np.array([0.0, 3.0, 3.0])
    controls = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 2.0]])
    result = rollout(q0, controls, parameters)
    np.testing.assert_allclose(result[1], [0.1, 3.0, 3.0])
    np.testing.assert_allclose(result[2], [0.1, 2.9, 3.2])


def test_mpc_returns_feasible_controls_for_stationary_target() -> None:
    parameters = MPCParameters(horizon=3, max_iterations=40)
    q = np.array([0.0, 3.1, 3.3])
    target = foot_position(q)
    first, controls = solve_mpc(
        q=q,
        foot_target=target,
        q_nominal=q,
        mpc_parameters=parameters,
    )
    assert first.shape == (3,)
    assert controls.shape == (3, 3)
    assert np.all(np.isfinite(controls))
    assert np.max(np.abs(controls)) <= np.max(parameters.velocity_limit) + 1e-9
