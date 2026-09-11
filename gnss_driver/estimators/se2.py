"""Protocol-independent 2-D rigid transform fitting."""
import numpy as np

def fit_rigid_2d(source, target):
    """Return ``(R, t)`` minimizing ||R*source+t-target||²."""
    source = np.asarray(source, dtype=float)
    target = np.asarray(target, dtype=float)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 2 or len(source) < 2:
        raise ValueError('source and target must be Nx2 arrays with at least two samples')
    src_center = source.mean(axis=0)
    dst_center = target.mean(axis=0)
    u, _, vt = np.linalg.svd((source-src_center).T @ (target-dst_center))
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T
    return rotation, dst_center - rotation @ src_center


def fit_rigid_2d_weighted(source, target, covariances=None, huber_delta=2.5, max_iterations=20, tol=1e-8):
    """Return ``(R, t)`` minimizing robust Mahalanobis error with covariances.

    Uses an initial estimate from closed-form SVD (fit_rigid_2d), followed by
    robust Huber M-estimation via Gauss-Newton on centered coordinates.

    Residual definition:
        r_i(θ, t) = target_i - (R(θ) * source_i + t)
    Jacobian:
        J_i = ∂r_i / ∂[θ, t] = [-D_i, -I_2], where D_i = d(R*s_i)/dθ
    Gauss-Newton step:
        H = Σ J_i^T W_i J_i
        g = - Σ J_i^T W_i r_i
        (H + λ I) Δx = g
        x ← x + Δx
    """
    import math

    source = np.asarray(source, dtype=float)
    target = np.asarray(target, dtype=float)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 2 or len(source) < 2:
        raise ValueError('source and target must be Nx2 arrays with at least two samples')

    src_center = source.mean(axis=0)
    dst_center = target.mean(axis=0)
    s_centered = source - src_center
    d_centered = target - dst_center

    rotation_init, _ = fit_rigid_2d(s_centered, d_centered)
    theta = math.atan2(rotation_init[1, 0], rotation_init[0, 0])
    translation_c = np.zeros(2, dtype=float)

    n_samples = len(source)
    if covariances is None:
        covariances = [None] * n_samples

    for _ in range(max(1, int(max_iterations))):
        cos_theta, sin_theta = math.cos(theta), math.sin(theta)
        current_rotation = np.array([[cos_theta, -sin_theta],
                                     [sin_theta, cos_theta]], dtype=float)

        H = np.zeros((3, 3), dtype=float)
        g = np.zeros(3, dtype=float)

        for s_pt, d_pt, cov in zip(s_centered, d_centered, covariances):
            pred = current_rotation @ s_pt + translation_c
            res = d_pt - pred

            # D = d(R*s)/dθ = [-sinθ*x - cosθ*y, cosθ*x - sinθ*y]
            D = np.array([
                -sin_theta * s_pt[0] - cos_theta * s_pt[1],
                 cos_theta * s_pt[0] - sin_theta * s_pt[1]
            ], dtype=float)

            if cov is None:
                W = np.eye(2)
            else:
                world_cov = current_rotation @ cov @ current_rotation.T
                W = np.linalg.pinv(world_cov)

            m2 = float(res.T @ W @ res)
            m = math.sqrt(max(0.0, m2))
            robust_w = 1.0 if m <= huber_delta else huber_delta / m
            W_eff = W * robust_w

            # Jacobian J = [-D, -I_2], J^T = [-D^T; -I_2]
            # H += J^T W J, g += - J^T W r
            J = np.column_stack((-D, -np.eye(2)))
            H += J.T @ W_eff @ J
            g += - J.T @ W_eff @ res

        try:
            delta = np.linalg.solve(H + 1e-6 * np.eye(3), g)
        except np.linalg.LinAlgError:
            break

        # Guard against divergence
        if not np.all(np.isfinite(delta)) or np.linalg.norm(delta[1:]) > 10.0:
            break

        theta += float(delta[0])
        translation_c += delta[1:]

        if np.linalg.norm(delta) < tol:
            break

    rotation_final = np.array([[math.cos(theta), -math.sin(theta)],
                               [math.sin(theta), math.cos(theta)]], dtype=float)
    translation_final = dst_center + translation_c - rotation_final @ src_center
    return rotation_final, translation_final


