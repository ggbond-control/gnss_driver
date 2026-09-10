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

