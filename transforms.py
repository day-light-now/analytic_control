from __future__ import annotations
import math
import numpy as np
def normalize_quat_xyzw(quat_xyzw: np.ndarray) -> np.ndarray:
    quat = np.asarray(quat_xyzw, dtype=float).reshape(4)
    norm = float(np.linalg.norm(quat))
    if norm <= 1e-12:
        raise ValueError("quaternion norm is too small")
    return quat / norm
def rotation_matrix_from_quat_xyzw(quat_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = normalize_quat_xyzw(quat_xyzw)
    return np.array([
        [1.0 - 2.0 * (y*y + z*z), 2.0 * (x*y - z*w), 2.0 * (x*z + y*w)],
        [2.0 * (x*y + z*w), 1.0 - 2.0 * (x*x + z*z), 2.0 * (y*z - x*w)],
        [2.0 * (x*z - y*w), 2.0 * (y*z + x*w), 1.0 - 2.0 * (x*x + y*y)],
    ], dtype=float)
def quat_xyzw_from_rotation_matrix(rotation: np.ndarray) -> np.ndarray:
    r = np.asarray(rotation, dtype=float).reshape(3, 3)
    trace = float(np.trace(r))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        quat = (
            (r[2, 1] - r[1, 2]) / s,
            (r[0, 2] - r[2, 0]) / s,
            (r[1, 0] - r[0, 1]) / s,
            0.25 * s,
        )
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        quat = (0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s)
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        quat = ((r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s)
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        quat = ((r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s, (r[1, 0] - r[0, 1]) / s)
    return normalize_quat_xyzw(np.array(quat, dtype=float))
def assert_rotation_matrix(rotation: np.ndarray, atol: float = 1e-6) -> None:
    r = np.asarray(rotation, dtype=float).reshape(3, 3)
    if not np.allclose(r.T @ r, np.eye(3), atol=atol):
        raise ValueError("rotation matrix is not orthonormal")
    determinant = float(np.linalg.det(r))
    if not np.isclose(determinant, 1.0, atol=atol):
        raise ValueError(f"rotation matrix determinant is {determinant}, expected +1")
def rotation_matrix_from_rpy_deg(rpy_deg: tuple[float, float, float]) -> np.ndarray:
    r, p, y = (math.radians(v) for v in rpy_deg)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr],
    ], dtype=float)
def rpy_deg_from_rotation_matrix(rotation: np.ndarray) -> tuple[float, float, float]:
    """Inverse of rotation_matrix_from_rpy_deg (ZYX / URDF RPY).

    R = Rz(yaw) @ Ry(pitch) @ Rx(roll). At gimbal lock (|pitch| = 90 deg) roll
    and yaw are not separable; this function then picks yaw = 0 and folds the
    remaining rotation into roll, so the returned tuple always rebuilds the same
    matrix. Callers must compare rotations as matrices, never as Euler strings.
    """
    r = np.asarray(rotation, dtype=float).reshape(3, 3)
    assert_rotation_matrix(r)
    sin_pitch = max(-1.0, min(1.0, -float(r[2, 0])))
    pitch = math.asin(sin_pitch)
    if abs(math.cos(pitch)) < 1e-9:
        yaw = 0.0
        roll = math.atan2(r[0, 1], r[1, 1]) if sin_pitch > 0.0 else math.atan2(-r[0, 1], r[1, 1])
    else:
        roll = math.atan2(r[2, 1], r[2, 2])
        yaw = math.atan2(r[1, 0], r[0, 0])
    return tuple(_clean_deg(math.degrees(value)) for value in (roll, pitch, yaw))


def _clean_deg(degrees: float) -> float:
    return 0.0 if abs(degrees) < 1e-9 else round(degrees, 9)


def transform_from_rotation_and_translation(rotation: np.ndarray, translation_m) -> np.ndarray:
    assert_rotation_matrix(rotation)
    t = np.eye(4, dtype=float)
    t[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
    t[:3, 3] = np.asarray(translation_m, dtype=float).reshape(3)
    return t
def transform_from_translation(translation_m) -> np.ndarray:
    return transform_from_rotation_and_translation(np.eye(3, dtype=float), translation_m)
def invert_transform(transform: np.ndarray) -> np.ndarray:
    transform = np.asarray(transform, dtype=float).reshape(4, 4)
    r, p = transform[:3, :3], transform[:3, 3]
    inv = np.eye(4, dtype=float)
    inv[:3, :3] = r.T
    inv[:3, 3] = -(r.T @ p)
    return inv
