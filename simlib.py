# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:26:55 2017

@author: kawalab
"""

import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, pow
import print_result as pr


def imput_gain(kp, kv, ki):
    gain = [kp, kv, ki]

    return gain


def PID_angle_control(gain, qd, q, dot_qd, dot_q, sum_q):
    Tau = []
    for i in range(len(gain)):
        kp, kv, ki = gain[i-1]
        tau = kp*(qd-q[i-1]) + kv*(dot_qd - dot_q[i-1]) + ki*sum_q[i-1]
        Tau.append(tau)

    return Tau


def PID_potiton_control_3dof(gain, Xd, X, Jt, dot_theta, sum_X):
    kp = []
    kv = []
    ki = []

    for i in range(len(gain)):
        Kp, Kv, Ki = gain[i-1]
        kp.append(Kp)
        kv.append(Kv)
        ki.append(Ki)

    tau1 = kp[0] * (Jt[0] * (Xd[0] - X[0]) + Jt[1] * (Xd[1] - X[1])) - kv[0] * dot_theta[0] + ki[0] * (Jt[0] * sum_X[0] + Jt[1] * sum_X[1])
    tau2 = kp[1] * (Jt[2] * (Xd[0] - X[0]) + Jt[3] * (Xd[1] - X[1])) - kv[1] * dot_theta[1] + ki[1] * (Jt[2] * sum_X[0] + Jt[3] * sum_X[1])
    tau3 = kp[2] * (Jt[4] * (Xd[0] - X[0]) + Jt[5] * (Xd[1] - X[1])) - kv[2] * dot_theta[2] + ki[2] * (Jt[4] * sum_X[0] + Jt[5] * sum_X[1])
    tau4 = kp[3] * (Jt[6] * (Xd[0] - X[0]) + Jt[7] * (Xd[1] - X[1])) - kv[3] * dot_theta[3] + ki[3] * (Jt[6] * sum_X[0] + Jt[7] * sum_X[1])

    Tau = [tau1, tau2, tau3, tau4]

    return Tau


def moment_inertia(m, l):
    moment = m*l*l/12

    return moment


def link_inertia(m, l, Inertia):
    for i in range(len(m)):
        inertia = moment_inertia(m[i-1], l[i-1])
        Inertia.append(inertia)

    return Inertia


def sum_angle_difference(sum_angle, qd, q, sampling_time):
    for i in range(len(sum_angle)):
        for i in range(len(q)):
            sum_angle[i-1] += (qd[i-1]-q[i-1]) * sampling_time

    return sum_angle


def sum_position_difference(sum_x, xd, x, sampling_time):
        sum_x += (xd - x)*sampling_time

        return sum_x


def calculate_angular_acceleration(Minv1, Minv2, tau1, tau2,
                                   h1, h2, G1, G2, D, dot_q):
    ddot_q = Minv1*(tau1 - h1 - G1 - D*dot_q) + Minv2*(tau2 - h2 - G2 - D*dot_q)

    return ddot_q


def angular_acceleration_3dof(inv_phi, f, A):
    ddot_q1 = inv_phi[0][0] * f[0] + inv_phi[0][1] * f[1] + inv_phi[0][2] * f[2] + inv_phi[0][3] * f[3] + inv_phi[0][4] * A[0] + inv_phi[0][5] * A[1]
    ddot_q2 = inv_phi[1][0] * f[0] + inv_phi[1][1] * f[1] + inv_phi[1][2] * f[2] + inv_phi[1][3] * f[3] + inv_phi[1][4] * A[0] + inv_phi[1][5] * A[1]
    ddot_q3 = inv_phi[2][0] * f[0] + inv_phi[2][1] * f[1] + inv_phi[2][2] * f[2] + inv_phi[2][3] * f[3] + inv_phi[2][4] * A[0] + inv_phi[2][5] * A[1]
    ddot_q4 = inv_phi[3][0] * f[0] + inv_phi[3][1] * f[1] + inv_phi[3][2] * f[2] + inv_phi[3][3] * f[3] + inv_phi[3][4] * A[0] + inv_phi[3][5] * A[1]

    ddot_q = [ddot_q1, ddot_q2, ddot_q3, ddot_q4]

    return ddot_q


def motor_angular_acceleration(Mm, tau, B, dot_theta, F=0):
    for i in range(len(tau)):
        ddot_theta = (tau[i-1] - B*dot_theta[i-1] - F[i-1])/Mm

    return ddot_theta


def EulerMethod(q, dot_q, ddot_q, sampling_time):
    for i in range(len(q)):
        dot_q[i-q] += ddot_q[i-1] * sampling_time
        q[i-1] += dot_q[i-1] * sampling_time

    return q, dot_q, ddot_q


def moment_matrix_3dof(m, l, lg, I, q):
    M1 = m[0] * lg[0] * lg[0] + I[0] + m[1] * (l[0] * l[0] + lg[1] * lg[1] + 2.0 * l[0] * lc[1] * cos(q[1])) + I[1]
    M2 = m[1] * (lg[1] * lg[1] + l[0] * lg[1] * cos(q[1])) + I[1]
    M3 = m[1] * (lg[1] * lg[1] + l[0] * lg[1] * cos(q[1])) + I[1]
    M4 = m[1] * lg[1] * lg[1] + I[1]

    M5 = m[2] * lg[2] * lg[2] + I[2] + m[3] * (l[2] * l[2] + lg[3] * lg[3] + 2.0 * l[2] * lc[3] * cos(q[2])) + I[3]
    M6 = m[3] * (lg[3] * lg[3] + l[2] * lg[3] * cos(q[3])) + I[3]
    M7 = m[3] * (lg[3] * lg[3] + l[2] * lg[3] * cos(q[3])) + I[3]
    M8 = m[3] * lg[3] * lg[3] + I[3]

    moment_matrix = [M1, M2, M3, M4, M5, M6, M7, M8]

    return moment_matrix


def twice_differential_values(l, q):
    E1 = -l[0] * sin(q[0]) - l[1] * sin(q[0] + q[1])
    E2 = -l[1] * sin(q[0] + q[1])
    E3 = l[2] * sin(q[2]) + l[3] * sin(q[2] + q[3])
    E4 = l[3] * sin(q[2] + q[3])

    E5 = l[0] * cos(q[0]) + l[1] * cos(q[0] + q[1])
    E6 = l[1] * cos(q[0] + q[1])
    E7 = -l[2] * cos(q[2]) - l[3] * cos(q[2] + q[3])
    E8 = -l[3] * cos(q[2] + q[3])

    E = [E1, E2, E3, E4, E5, E6, E7, E8]

    return E


def phi_matrix(M, E):
    phi1, phi2, phi3, phi4, phi5, phi6 = M[0], M[1], 0, 0, E[0], E[4]
    phi7, phi8, phi9, phi10, phi11, phi12 = M[2], M[3], 0, 0, E[1], E[5]
    phi13, phi14, phi15, phi16, phi17, phi18 = 0, 0, M[4], M[5], E[2], E[6]
    phi19, phi20, phi21, phi22, phi23, phi24 = 0, 0, M[6], M[7], E[3], E[7]
    phi25, phi26, phi27, phi28, phi29, phi30 = E[0], E[1], E[2], E[3], 0, 0
    phi31, phi32, phi33, phi34, phi35, phi36 = E[5], E[6], E[7], E[8], 0, 0

    Phi = [[phi1, phi2, phi3, phi4, phi5, phi6],
           [phi7, phi8, phi9, phi10, phi11, phi12],
           [phi13, phi14, phi15, phi16, phi17, phi18],
           [phi19, phi20, phi21, phi22, phi23, phi24],
           [phi25, phi26, phi27, phi28, phi29, phi30],
           [phi31, phi32, phi33, phi34, phi35, phi36]]

    return Phi


def coriolis_item_3dof(m, l, lg, I, q, dot_q):
    h1 = -m[1] * l[0] * lg[1] * (2 * dot_q[0] + dot_q[1]) * dot_q[1] * sin(q[1])
    h2 = m[1] * l[0] * lg[1] * dot_q[0] * dot_q[0] * sin(q[1])
    h3 = -m[3] * l[2] * lg[3] * (2 * dot_q[2] + dot_q[3]) * dot_q[3] * sin(q[3])
    h4 = m[3] * l[2] * lg[3] * dot_q[2] * dot_q[2] * sin(q[3])

    coriori_items = [h1, h2, h3, h4]

    return coriori_items


def link_imput_force(tau, h, B, dot_q):
    imput_force = []
    for i in range(len(tau)):
        f = tau[i-1] - h[i-1] - B * dot_q[i-1]
        imput_force.append(f)

    return imput_force


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


def difference_part(theta, q):
    e_data = []
    for i in range(len(theta)):
        e = (theta[i-1] - q[i-1])
        e_data.append(e)

    return e_data


def non_linear_item(k1, k2, e, K):
    for i in range(len(e)):
        k = k1 + k2*pow(e[i], 2)
        k = k*e[i]
        K.append(k)

    return K


def restraint_part(l, q, dot_q):
    dot_P = -l[0] * sin(q[0]) * dot_q[0] - l[1] * sin(q[0] + q[1]) * (dot_q[0] + dot_q[1]) + l[2] * sin(q[2]) * dot_q[2] + l[3] * sin(q[2] + q[3]) * (dot_q[2] + dot_q[3])
    P = l[0] * cos(q[0]) + l[1] * cos(q[0] + q[1]) - l[2] * cos(q[2]) - l[3] * cos(q[2] + q[3])

    dot_Q = l[0] * cos(q[0]) * dot_q[0] + l[1] * cos(q[0] + q[1]) * (dot_q[0] + dot_q[1]) - l[2] * cos(q[2]) * dot_q[2] - l[3] * cos(q[2] + q[3]) * (dot_q[2] + dot_q[3])
    Q = l[0] * sin(q[0]) + l[1] * sin(q[0] + q[1]) - l[2] * sin(q[2]) - l[3] * sin(q[2] + q[3])

    return dot_P, P, dot_Q, Q


def input_forces(l, q, dot_q, h, D, K, Jt, P, Q, dot_P, dot_Q, Fx=0, Fy=0, s=1):
    f1 = K[0] + Jt[0] * Fx + Jt[1] * Fy - h[0] - D * dot_q[0]
    f2 = K[1] + Jt[2] * Fx + Jt[3] * Fy - h[1] - D * dot_q[1]
    f3 = K[2] + Jt[4] * Fx + Jt[5] * Fy - h[2] - D * dot_q[2]
    f4 = K[3] + Jt[6] * Fx + Jt[7] * Fy - h[3] - D * dot_q[3]

    A1 = -2 * s[0] * dot_P - s[0] * s[0] * P + l[0] * cos(q[0]) * dot_q[0] * dot_q[0] + l[1] * cos(q[0] + q[1]) * (dot_q[0] + dot_q[1]) * (dot_q[0] + dot_q[1]) - l[2] * cos(q[2]) * dot_q[2] * dot_q[2] - l[3] * cos(q[2] + q[3]) * (dot_q[2] + dot_q[3]) * (dot_q[2] + dot_q[3])
    A2 = -2 * s[1] * dot_Q - s[1] * s[1] * Q + l[0] * sin(q[0]) * dot_q[0] * dot_q[0] + l[1] * sin(q[0] + q[1]) * (dot_q[0] + dot_q[1]) * (dot_q[0] + dot_q[1]) - l[2] * sin(q[2]) * dot_q[2] * dot_q[2] - l[3] * sin(q[2] + q[3]) * (dot_q[2] + dot_q[3]) * (dot_q[2] + dot_q[3])

    f = [f1, f2, f3, f4]
    A = [A1, A2]

    return f, A


def binding_force(inv_phi, f, A):
    lambda_x = inv_phi[4][0] * f[0] + inv_phi[4][1] * f[1] + inv_phi[4][2] * f[2] + inv_phi[4][3] * f[3] + inv_phi[4][4] * A[0] + inv_phi[4][5] * A[1]
    lambda_y = inv_phi[5][0] * f[0] + inv_phi[5][1] * f[1] + inv_phi[5][2] * f[2] + inv_phi[5][3] * f[3] + inv_phi[5][4] * A[0] + inv_phi[5][5] * A[1]

    lam = [lambda_x, lambda_y]

    return lam


def inverse_matrix(Matrix):
    Matrix = np.array(Matrix)
    Inverse_Matorix = np.linalg.inv(Matrix)

    return Inverse_Matorix


def transpose_matrix(matrix):
    Matrix = np.array(matrix)
    transpose_matrix = np.transpose(Matrix)

    return transpose_matrix


def non_linear_parameta(k1, k2):
    K = [k1, k2]

    return K


def jacobi_matrix(l, q):
    J1 = -l[0] * sin(q[0]) - l[1] * sin(q[0] + q[1])
    J2 = -l[1] * sin(q[0] + q[1])
    J3 = l[0] * cos(q[0]) + l[1] * cos(q[0] + q[1])
    J4 = l[1] * cos(q[0] + q[1])

    J5 = -l[2] * sin(q[2]) - l[3] * sin(q[2] + q[3])
    J6 = -l[3] * sin(q[2] + q[3])
    J7 = l[2] * cos(q[2]) + l[3] * cos(q[2] + q[3])
    J8 = l[3] * cos(q[2] + q[3])

    J = [[J1, J2, J3, J4], [J5, J6, J7, J8]]

    return J


def print_non_lineaar_Characteristics():
    k1 = [0.001, 10]
    k2 = [0.001, 0.005, 0.01]
    x_data = range(-50, 50)
    y1_data = []
    y2_data = []
    y3_data = []
    y4_data = []

    y1_data = non_linear_item(k1[0], k2[0], x_data, y1_data)
    y2_data = non_linear_item(k1[0], k2[1], x_data, y2_data)
    y3_data = non_linear_item(k1[0], k2[2], x_data, y3_data)
    y4_data = non_linear_item(k1[1], k2[0], x_data, y4_data)

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


if __name__ == '__main__':
    print_non_lineaar_Characteristics()

    example_matrix = [[1, 2, 3], [1, 1, 1], [1, 3, 2]]
    invm = inverse_matrix(example_matrix)
    print(invm)
