import math
import numpy as np
import pytest

from gnss_driver.estimators.se2 import fit_rigid_2d, fit_rigid_2d_weighted


def test_fit_rigid_2d_exact():
    theta = 0.5
    R_true = np.array([[math.cos(theta), -math.sin(theta)],
                       [math.sin(theta), math.cos(theta)]])
    t_true = np.array([12.3, -45.6])

    source = np.array([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0], [2.0, 3.0]])
    target = (R_true @ source.T).T + t_true

    R_est, t_est = fit_rigid_2d(source, target)
    np.testing.assert_allclose(R_est, R_true, atol=1e-10)
    np.testing.assert_allclose(t_est, t_true, atol=1e-10)


def test_fit_rigid_2d_weighted_exact():
    theta = -0.15
    R_true = np.array([[math.cos(theta), -math.sin(theta)],
                       [math.sin(theta), math.cos(theta)]])
    t_true = np.array([0.05, -0.08])

    source = np.array([[1.0, 2.0], [3.0, 5.0], [10.0, -2.0], [4.0, 7.0]])
    target = (R_true @ source.T).T + t_true

    R_est, t_est = fit_rigid_2d_weighted(source, target, max_iterations=20)
    np.testing.assert_allclose(R_est, R_true, atol=1e-7)
    np.testing.assert_allclose(t_est, t_true, atol=1e-7)


def test_weighted_fit_outlier_rejection():
    """Verify that Huber weighting rejects outliers and achieves higher accuracy than unweighted SVD."""
    np.random.seed(42)
    theta_true = 0.0818
    R_true = np.array([[math.cos(theta_true), -math.sin(theta_true)],
                       [math.sin(theta_true), math.cos(theta_true)]])
    t_true = np.array([0.0277, -0.0593])

    N = 100
    t_pts = np.linspace(0, 20, N)
    source = np.column_stack([t_pts, 2.0 * np.sin(0.3 * t_pts)])
    noise = np.random.normal(0, 0.02, (N, 2))

    # Add extreme outliers
    noise[10] += [2.0, -2.0]
    noise[50] += [-3.0, 1.5]
    target = (R_true @ source.T).T + t_true + noise

    covs = [np.diag([0.02**2, 0.02**2]) for _ in range(N)]
    covs[10] = np.diag([1.0**2, 1.0**2])
    covs[50] = np.diag([1.0**2, 1.0**2])

    R_unw, t_unw = fit_rigid_2d(source, target)
    R_w, t_w = fit_rigid_2d_weighted(source, target, covariances=covs, huber_delta=2.0, max_iterations=20)

    err_t_unw = np.linalg.norm(t_unw - t_true)
    err_t_w = np.linalg.norm(t_w - t_true)

    # Weighted fit with Huber loss must be more accurate than unweighted
    assert err_t_w < err_t_unw
    assert err_t_w < 0.01  # Millimeter precision despite large outliers


def test_no_divergence_across_many_iterations():
    """Regression test: verify that running 15 to 50 iterations never diverges."""
    np.random.seed(123)
    theta_true = 0.081844
    R_true = np.array([[math.cos(theta_true), -math.sin(theta_true)],
                       [math.sin(theta_true), math.cos(theta_true)]])
    t_true = np.array([0.02778, -0.05931])

    N = 200
    t_pts = np.linspace(0, 15, N)
    source = np.column_stack([t_pts, np.sin(0.2 * t_pts)])
    noise = np.random.normal(0, 0.01, (N, 2))
    target = (R_true @ source.T).T + t_true + noise
    covs = [np.diag([0.02**2, 0.02**2]) for _ in range(N)]

    for iters in [5, 15, 30, 50]:
        R_est, t_est = fit_rigid_2d_weighted(source, target, covariances=covs, max_iterations=iters)
        # Translation must remain within centimeters of true, never thousands of meters
        np.testing.assert_allclose(t_est, t_true, atol=0.01)
        np.testing.assert_allclose(R_est, R_true, atol=0.01)


def test_invalid_inputs():
    with pytest.raises(ValueError):
        fit_rigid_2d([[1, 2]], [[1, 2]])
    with pytest.raises(ValueError):
        fit_rigid_2d_weighted([[1, 2]], [[1, 2]])
    with pytest.raises(ValueError):
        fit_rigid_2d_weighted([[1, 2], [3, 4]], [[1, 2, 3], [4, 5, 6]])

