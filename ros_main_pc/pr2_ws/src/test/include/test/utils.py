#! /usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation


def smoothing(smooth_cmd, real_cmd):
    if smooth_cmd is None:
        smooth_cmd = real_cmd
    else:
        a = 0.95
        first_term = [x*a for x in smooth_cmd]
        second_term = [x*(1-a) for x in real_cmd]
        smooth_cmd = [x+y for x,y in zip(first_term, second_term)]
    return smooth_cmd

"""
q = [x, y, z, w]
"""
def quaternion_to_conjugate(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])

def rot_mat_to_quaternion(self, rot_mat):
    return Rotation.from_matrix(rot_mat).as_quat()

def quaternion_multiply(self, q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([x, y, z, w])