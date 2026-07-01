import numpy as np
from scipy.spatial.transform import Rotation as R

def quaternion_multiply(q1, q2):
    """Hamilton product of two quaternions [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def quaternion_to_rotation_matrix(q):
    """Convert quaternion to 3x3 rotation matrix (body to inertial)."""
    return R.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()  # scipy uses [x,y,z,w]

def normalize_quaternion(q):
    return q / np.linalg.norm(q)

def skew_symmetric(v):
    """Skew-symmetric matrix for cross product."""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])
