# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:26:55 2017

@author: kawalab
"""

import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, degrees, radians


def PIDcontrol(kp, kv, ki, qd, q, dot_qd, dot_q, sum_q):
    tau = kp*(qd-q) + kv*(dot_qd - dot_q) + ki*sum_q

    return tau


def sum_angle_difference(qd, q, sampling_time):
    sum_angle = (qd-q) * sampling_time

    return sum_angle


def calculate_angular_acceleration(Minv1, Minv2, tau1, tau2,
                                   h1, h2, G1, G2):
    ddot_q = Minv1*(tau1 - h1 - G1) + Minv2*(tau2 - h2 - G2)

    return ddot_q


def EulerMethod(q1, q2, dot_q1, dot_q2, ddot_q1, ddot_q2, sampling_time):
    dot_q1 += ddot_q1 * sampling_time
    dot_q2 += ddot_q2 * sampling_time
    q1 += dot_q1 * sampling_time
    q2 += dot_q2 * sampling_time

    return q1, q2, dot_q1, dot_q2


def invm_2dof(M1, M2, M3, M4):
    detM = M1*M4 - M2*M3
    Minv1 = M4/detM
    Minv2 = -1.0*M3/detM
    Minv3 = -1.0*M2/detM
    Minv4 = M1/detM

    return Minv1, Minv2, Minv3, Minv4


def inertia_term_2dof(m1, m2, l1, l2, lg1, lg2, I1, I2, q2):
    M1 = m1*lg1*lg1 + m2*l1*l1 + m2*lg2*lg2 + I1 + I2 + 2*m2*l1*lg2*cos(q2)
    M2 = m2*lg2*lg2 + I2 + m2*l1*lg2*cos(q2)
    M3 = m2*lg2*lg2 + I2 + m2*l2*lg2*cos(q2)
    M4 = m2*lg2*lg2 + I2

    return M1, M2, M3, M4


def coriolis_item_2dof(m2, l1, lg2, q2, dot_q1, dot_q2):
    h1 = -m2*l1*lg2*(2*dot_q1 + dot_q2)*dot_q2*sin(q2)
    h2 = m2*l1*lg2*dot_q1*dot_q1*sin(q2)

    return h1, h2


def gravity_item_2dof(m1, m2, l1, lg1, lg2, q1, q2, g):
    g1 = (m1*g*lg1 + m2*g*l1)*cos(q1) + m2*g*lg2*cos(q1+q2)
    g2 = m2*g*lg2*cos(q1+q2)

    return g1, g2


def moment_inertia(m, l):
    moment = m*l*l/12

    return moment


def simulation_time(count_time, sampling_time):
    simulation_time = count_time/sampling_time

    return simulation_time


def save_log(x_data, x_data_log):
    x_data_log.append(x_data)

    return x_data_log


if __name__ == '__main__':
    PIDcontrol()