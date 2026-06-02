import math

import numpy as np


def quaternion_matrix(quaternion):
    x, y, z, w = quaternion
    n = x * x + y * y + z * z + w * w
    if n < np.finfo(float).eps:
        return np.identity(4)
    s = 2.0 / n
    xs, ys, zs = x * s, y * s, z * s
    wx, wy, wz = w * xs, w * ys, w * zs
    xx, xy, xz = x * xs, x * ys, x * zs
    yy, yz, zz = y * ys, y * zs, z * zs
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy, 0.0],
        [xy + wz, 1.0 - (xx + zz), yz - wx, 0.0],
        [xz - wy, yz + wx, 1.0 - (xx + yy), 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def quaternion_from_euler(ai, aj, ak):
    ai *= 0.5
    aj *= 0.5
    ak *= 0.5
    ci, si = math.cos(ai), math.sin(ai)
    cj, sj = math.cos(aj), math.sin(aj)
    ck, sk = math.cos(ak), math.sin(ak)
    return np.array([
        si * cj * ck - ci * sj * sk,
        ci * sj * ck + si * cj * sk,
        ci * cj * sk - si * sj * ck,
        ci * cj * ck + si * sj * sk,
    ])


def euler_from_quaternion(quaternion):
    x, y, z, w = quaternion
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def euler_from_matrix(matrix):
    matrix = np.asarray(matrix, dtype=float)
    m = matrix[:3, :3]
    sy = math.sqrt(m[0, 0] * m[0, 0] + m[1, 0] * m[1, 0])
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(m[2, 1], m[2, 2])
        pitch = math.atan2(-m[2, 0], sy)
        yaw = math.atan2(m[1, 0], m[0, 0])
    else:
        roll = math.atan2(-m[1, 2], m[1, 1])
        pitch = math.atan2(-m[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def quaternion_from_matrix(matrix):
    matrix = np.asarray(matrix, dtype=float)
    m = matrix[:4, :4] if matrix.shape[0] >= 4 and matrix.shape[1] >= 4 else np.identity(4)
    if matrix.shape[0] < 4 or matrix.shape[1] < 4:
        m[:3, :3] = matrix[:3, :3]
    q = np.empty((4,), dtype=float)
    t = np.trace(m)
    if t > m[3, 3]:
        q[3] = t
        q[2] = m[1, 0] - m[0, 1]
        q[1] = m[0, 2] - m[2, 0]
        q[0] = m[2, 1] - m[1, 2]
    else:
        i, j, k = 0, 1, 2
        if m[1, 1] > m[0, 0]:
            i, j, k = 1, 2, 0
        if m[2, 2] > m[i, i]:
            i, j, k = 2, 0, 1
        q[i] = m[i, i] - m[j, j] - m[k, k] + m[3, 3]
        q[j] = m[j, i] + m[i, j]
        q[k] = m[k, i] + m[i, k]
        q[3] = m[k, j] - m[j, k]
    q *= 0.5 / math.sqrt(t * m[3, 3] if t > m[3, 3] else m[i, i] - m[j, j] - m[k, k] + m[3, 3])
    return q


def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def quaternion_conjugate(quaternion):
    x, y, z, w = quaternion
    return np.array([-x, -y, -z, w])


def quaternion_inverse(quaternion):
    quaternion = np.asarray(quaternion, dtype=float)
    return quaternion_conjugate(quaternion) / np.dot(quaternion, quaternion)


def quaternion_about_axis(angle, axis):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < np.finfo(float).eps:
        return np.array([0.0, 0.0, 0.0, 1.0])
    axis = axis / norm
    sina2 = math.sin(angle / 2.0)
    return np.array([axis[0] * sina2, axis[1] * sina2, axis[2] * sina2, math.cos(angle / 2.0)])
