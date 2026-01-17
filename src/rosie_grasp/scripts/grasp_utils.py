"""Dependency-free geometry and candidate-selection helpers for the grasp node."""
import numpy as np


def normalize_quaternion(quaternion):
    q = np.asarray(quaternion, dtype=float).reshape(-1)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError("quaternion must contain four finite values")
    norm = np.linalg.norm(q)
    if norm < 1e-9:
        raise ValueError("quaternion norm is zero")
    return q / norm


def depth_to_point_cloud(depth_m, camera_matrix, min_depth=0.15, max_depth=2.0,
                         bounds=None, mask=None):
    """Project a depth image into metric optical-frame XYZ points.

    ``bounds`` is [xmin, xmax, ymin, ymax, zmin, zmax] in metres. ``mask`` is
    an optional image-sized boolean mask (for example, a YOLO target crop).
    """
    depth = np.asarray(depth_m, dtype=float)
    k = np.asarray(camera_matrix, dtype=float).reshape(3, 3)
    if depth.ndim != 2 or not np.all(np.isfinite(k)) or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError("invalid depth image or camera matrix")
    rows, cols = np.indices(depth.shape)
    valid = np.isfinite(depth) & (depth >= min_depth) & (depth <= max_depth)
    if mask is not None:
        valid &= np.asarray(mask, dtype=bool)
    z = depth
    points = np.stack(((cols - k[0, 2]) * z / k[0, 0],
                       (rows - k[1, 2]) * z / k[1, 1], z), axis=-1)
    if bounds is not None:
        b = np.asarray(bounds, dtype=float).reshape(-1)
        if b.shape != (6,):
            raise ValueError("bounds must be [xmin, xmax, ymin, ymax, zmin, zmax]")
        valid &= ((points[..., 0] >= b[0]) & (points[..., 0] <= b[1]) &
                  (points[..., 1] >= b[2]) & (points[..., 1] <= b[3]) &
                  (points[..., 2] >= b[4]) & (points[..., 2] <= b[5]))
    return points[valid], valid


def select_candidate(candidates, confidence_threshold=0.5):
    """Return the highest-scoring finite, collision-free learned candidate."""
    valid = []
    for candidate in candidates or []:
        if not isinstance(candidate, dict):
            continue
        try:
            score = float(candidate.get("score", candidate.get("confidence", 0.0)))
            position = np.asarray(candidate["position"], dtype=float).reshape(-1)
            quaternion = normalize_quaternion(candidate["quaternion"])
        except (KeyError, TypeError, ValueError):
            continue
        if (score < confidence_threshold or position.shape != (3,) or
                not np.all(np.isfinite(position)) or
                not bool(candidate.get("collision_free", True))):
            continue
        result = dict(candidate)
        result["position"] = position
        result["quaternion"] = quaternion
        result["score"] = score
        valid.append(result)
    return max(valid, key=lambda item: item["score"]) if valid else None


def quaternion_multiply(lhs, rhs):
    x1, y1, z1, w1 = normalize_quaternion(lhs)
    x2, y2, z2, w2 = normalize_quaternion(rhs)
    return normalize_quaternion((w1*x2 + x1*w2 + y1*z2 - z1*y2,
                                 w1*y2 - x1*z2 + y1*w2 + z1*x2,
                                 w1*z2 + x1*y2 - y1*x2 + z1*w2,
                                 w1*w2 - x1*x2 - y1*y2 - z1*z2))


def rotate_vector(quaternion, vector):
    q = normalize_quaternion(quaternion)
    vx, vy, vz = map(float, vector)
    x, y, z, w = q
    # Equivalent to q * [v, 0] * conjugate(q), without normalizing the
    # intermediate pure-vector quaternion.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return np.asarray((vx + w * tx + y * tz - z * ty,
                       vy + w * ty + z * tx - x * tz,
                       vz + w * tz + x * ty - y * tx))


def offset_position(position, quaternion, distance, local_axis=(0.0, 0.0, -1.0)):
    """Offset along a learned gripper-local approach axis, not world Z."""
    axis = np.asarray(local_axis, dtype=float).reshape(3)
    norm = np.linalg.norm(axis)
    if norm < 1e-9:
        raise ValueError("local approach axis is zero")
    return np.asarray(position, dtype=float).reshape(3) + rotate_vector(quaternion, axis / norm) * float(distance)
