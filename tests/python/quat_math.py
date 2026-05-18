"""Quaternion helpers — mirror Fox Mocap's scr/main.cpp math exactly.
WXYZ (Hamilton) convention; numpy-only so tests run anywhere."""
import numpy as np

def qmul(a, b):
    """Hamilton product. Matches scr/main.cpp:quat_mult."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ])

def qnorm(q):
    n = np.linalg.norm(q)
    return q if n < 1e-12 else q / n

def qinv(q):
    return np.array([q[0], -q[1], -q[2], -q[3]]) / max(1e-12, float(np.dot(q, q)))

def qconj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

def qrot(q, v):
    """Rotate vector v by quaternion q. Matches scr/main.cpp:vec_rotate."""
    qv = np.array([q[1], q[2], q[3]])
    t = 2.0 * np.cross(qv, v)
    return v + q[0] * t + np.cross(qv, t)

def axangle(ax, rad):
    ax = np.array(ax, dtype=float)
    h = rad * 0.5
    s = np.sin(h)
    return qnorm(np.array([np.cos(h), ax[0]*s, ax[1]*s, ax[2]*s]))

def euler_xyz(a, b, c):
    """Intrinsic XYZ: Qx(a) * Qy(b) * Qz(c). Matches scr/main.cpp:euler_to_quat with seq='XYZ'."""
    return qnorm(qmul(qmul(axangle([1, 0, 0], a),
                            axangle([0, 1, 0], b)),
                       axangle([0, 0, 1], c)))

def mirror_y_quat(q):
    """Mirror a rotation quaternion through the world Y-plane.  Matches
    scr/main.cpp:mirror_y_quat — flips X and Z components."""
    return np.array([q[0], -q[1], q[2], -q[3]])

def yaw_only(q):
    """Extract twist about world Z.  Matches scr/main.cpp:yaw_only_quat."""
    w, z = q[0], q[3]
    if w < 0:
        w, z = -w, -z
    n2 = w * w + z * z
    if n2 < 1e-12:
        return np.array([1., 0, 0, 0])
    n = 1.0 / np.sqrt(n2)
    return np.array([w * n, 0, 0, z * n])

def swing_twist(q, axis_u):
    """Decompose q into swing (perpendicular to axis_u) and twist (about axis_u).
    Matches scr/main.cpp:swingTwistDecompose."""
    dot = q[1]*axis_u[0] + q[2]*axis_u[1] + q[3]*axis_u[2]
    tw = np.array([q[0], dot*axis_u[0], dot*axis_u[1], dot*axis_u[2]])
    n2 = float(np.dot(tw, tw))
    if n2 < 1e-12:
        return q, np.array([1., 0, 0, 0])
    tw = tw / np.sqrt(n2)
    if tw[0] < 0:
        tw = -tw
    swing = qnorm(qmul(q, qinv(tw)))
    return swing, tw

def qangle_deg(q):
    """Geodesic angle of rotation in degrees. Matches scr/main.cpp:quat_angle_deg."""
    w = min(1.0, abs(q[0]))
    return float(2.0 * np.degrees(np.arccos(w)))

def slerp(a, b, t):
    """Quaternion slerp.  Matches scr/main.cpp:slerp_quat."""
    dot = float(np.dot(a, b))
    b2 = b.copy()
    if dot < 0.0:
        b2 = -b
        dot = -dot
    if dot > 0.9995:
        return qnorm((1 - t) * a + t * b2)
    theta0 = np.arccos(dot)
    theta = theta0 * t
    s0 = np.sin(theta0 - theta) / np.sin(theta0)
    s1 = np.sin(theta) / np.sin(theta0)
    return qnorm(s0 * a + s1 * b2)

PI = np.pi
