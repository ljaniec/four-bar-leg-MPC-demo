import numpy as np

from four_bar_leg_mpc.controller import MPCParameters, objective, rollout, solve_mpc
from four_bar_leg_mpc.geometry import foot_position
from four_bar_leg_mpc.simulation import run_simulation


def test_rollout_integrates_joint_velocity_controls() -> None:
    parameters = MPCParameters(dt=0.1, horizon=2)
    q0 = np.array([0.0, 3.0, 3.0])
    controls = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 2.0]])
    result = rollout(q0, controls, parameters)
    np.testing.assert_allclose(result[1], [0.1, 3.0, 3.0])
    np.testing.assert_allclose(result[2], [0.1, 2.9, 3.2])


def test_mpc_returns_feasible_controls_for_stationary_setpoint() -> None:
    parameters = MPCParameters(horizon=3, max_iterations=40)
    q = np.array([0.0, 3.1, 3.3])
    foot_setpoint = foot_position(q)
    first, controls = solve_mpc(
        q=q,
        foot_setpoint=foot_setpoint,
        q_nominal=q,
        mpc_parameters=parameters,
    )
    assert first.shape == (3,)
    assert controls.shape == (3, 3)
    assert np.all(np.isfinite(controls))
    assert np.max(np.abs(controls)) <= np.max(parameters.velocity_limit) + 1e-9


def test_objective_uses_one_constant_cartesian_foot_setpoint() -> None:
    parameters = MPCParameters(horizon=2)
    q = np.array([0.0, 3.1, 3.3])
    controls = np.zeros((parameters.horizon, 3))
    foot_setpoint = foot_position(q)
    cost = objective(
        controls.ravel(),
        q,
        foot_setpoint,
        q,
        np.zeros(3),
        mpc_parameters=parameters,
    )
    assert cost == 0.0


def test_simulation_stores_a_single_fixed_setpoint() -> None:
    result = run_simulation(steps=2)
    assert result.foot_setpoint.shape == (3,)
    assert result.q_nominal.shape == (3,)
    np.testing.assert_allclose(result.foot_setpoint, foot_position(result.q_nominal))
