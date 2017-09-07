# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:26:55 2017

@author: kawalab
"""

import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, pow
import print_result as pr


def PIDcontrol(kp, kv, ki, qd, q, dot_qd, dot_q, sum_q):
    tau = kp*(qd-q) + kv*(dot_qd - dot_q) + ki*sum_q

    return tau


def sum_angle_difference(sum_angle, qd, q, sampling_time):
    sum_angle += (qd-q) * sampling_time

    return sum_angle


def sum_position_difference(sum_x, xd, x, sampling_time):
    sum_x += (xd - x)*sampling_time

    return sum_x


def calculate_angular_acceleration(Minv1, Minv2, tau1, tau2,
                                   h1, h2, G1, G2, D, dot_q):
    ddot_q = Minv1*(tau1 - h1 - G1 - D*dot_q) + Minv2*(tau2 - h2 - G2 - D*dot_q)

    return ddot_q


def motor_angular_acceleration(Mm, tau, B, dot_theta, F=0):
    ddot_theta = (tau - B*dot_theta - F)/Mm

    return ddot_theta


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


def difference_part(theta, q):
    e = (theta - q)

    return e


def non_linear_item(k1, k2, e):
    K = k1 + k2*pow(e, 2)

    return K


def print_non_lineaar_Characteristics():
    k1 = [0.001, 10]
    k2 = [0.001, 0.005, 0.01]
    x_data = []
    y1_data = []
    y2_data = []
    y3_data = []
    y4_data = []
    for i in range(-100, 100, 1):
        x = i
        K1 = non_linear_item(k1[0], k2[0], x)
        K2 = non_linear_item(k1[0], k2[1], x)
        K3 = non_linear_item(k1[0], k2[2], x)
        K4 = non_linear_item(k1[1], k2[0], x)

        y1 = K1*x
        y2 = K2*x
        y3 = K3*x
        y4 = K4*x

        x_data.append(x)
        y1_data.append(y1)
        y2_data.append(y2)
        y3_data.append(y3)
        y4_data.append(y4)

    yd = [y1_data, y2_data, y3_data, y4_data]
    label_name = ["k1 = {}, k2 = {}". format(k1[0], k2[0]),
                  "k1 = {}, k2 = {}". format(k1[0], k2[1]),
                  "k1 = {}, k2 = {}". format(k1[0], k2[2]),
                  "k1 = {}, k2 = {}". format(k1[1], k2[0])]
    plt.figure(figsize=(7, 3))
    pr.print_graph("Non Linear Characteristics",
                   x_data, yd, label_name, "X", "K", 4)
    plt.show()


def simulation_time(count_time, sampling_time):
    simulation_time = count_time/sampling_time

    return simulation_time


def save_log(x_data, x_data_log):
    x_data_log.append(x_data)

    return x_data_log


if __name__ == '__main__':
    print_non_lineaar_Characteristics()
