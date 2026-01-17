"""Small geometry helpers used by the MoveIt controller and unit tests."""
import math


def normalize_quaternion(q):
    norm = math.sqrt(sum(float(value) ** 2 for value in q))
    if len(q) != 4 or not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid quaternion")
    return tuple(float(value) / norm for value in q)


def rotate_vector(q, v):
    x, y, z, w = normalize_quaternion(q)
    vx, vy, vz = (float(value) for value in v)
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def offset_along_grasp_axis(position, quaternion, distance, local_axis=(0.0, 0.0, -1.0)):
    axis_norm = math.sqrt(sum(float(value) ** 2 for value in local_axis))
    if axis_norm < 1e-9:
        raise ValueError("local approach axis is zero")
    direction = rotate_vector(quaternion, tuple(float(value) / axis_norm for value in local_axis))
    return tuple(float(position[i]) + float(distance) * direction[i] for i in range(3))
