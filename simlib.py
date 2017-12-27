# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:26:55 2017

@author: kawalab
"""

import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, pow, atan, sqrt
import print_result as pr


def imput_gain(kp, kv, ki):
    gain = [kp, kv, ki]

    return gain


def unit_P_control(kp, x, xd):
    tau = kp*(xd-x)

    return tau


def PID_angle_control(gain, qd, q, dot_qd, dot_q, sum_q):
    Tau = []
    for i in range(len(gain)):
        kp, kv, ki = gain[i]
        tau = kp*(qd-q[i]) + kv*(dot_qd - dot_q[i]) + ki*sum_q[i]
        Tau.append(tau)

    return Tau


def PID_potiton_control_3dof(gain, Xd, X, Jt, dot_theta, sum_X):
    kp = []
    kv = []
    ki = []

    for i in range(len(gain)):
        Kp, Kv, Ki = gain[i]
        kp.append(Kp)
        kv.append(Kv)
        ki.append(Ki)

    tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1])) - kv[0] * dot_theta[0] + ki[0] * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1])
    tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1])) - kv[1] * dot_theta[1] + ki[1] * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1])
    tau3 = kp[2] * (Jt[2][0] * (Xd[0] - X[0]) + Jt[2][1] * (Xd[1] - X[1])) - kv[2] * dot_theta[2] + ki[2] * (Jt[2][0] * sum_X[0] + Jt[2][1] * sum_X[1])
    tau4 = kp[3] * (Jt[3][0] * (Xd[0] - X[0]) + Jt[3][1] * (Xd[1] - X[1])) - kv[3] * dot_theta[3] + ki[3] * (Jt[3][0] * sum_X[0] + Jt[3][1] * sum_X[1])

    Tau = [tau1, tau2, tau3, tau4]

    return Tau


def new_PID_position_control(gain, dot_theta, Xd, X, Jt, sum_X, Fconstant=0):
    kp = []
    kv = []
    ki = []

    for i in range(len(gain)):
        Kp, Kv, Ki = gain[i]
        kp.append(Kp)
        kv.append(Kv)
        ki.append(Ki)

    tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1])) - kv[0] * dot_theta[0] + ki[0] * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1]) + (Jt[0][0] * Fconstant + Jt[0][1] * Fconstant)
    tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1])) - kv[1] * dot_theta[1] + ki[1] * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1]) + (Jt[1][0] * Fconstant + Jt[1][1] * Fconstant)

    tau3 = 0
    tau4 = -(Jt[3][0] * Fconstant + Jt[3][1] * Fconstant)

    Tau = [tau1, tau2, tau3, tau4]

    return Tau


def new_PID_position_control_ver2(gain, dot_theta, Xd, X, Jt, sum_X, Fconstant=0):
    kp = []
    kv = []
    ki = []

    for i in range(len(gain)):
        Kp, Kv, Ki = gain[i]
        kp.append(Kp)
        kv.append(Kv)
        ki.append(Ki)

    tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1])) - kv[0] * dot_theta[0] + ki[0] * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1]) + (Jt[0][0] * Fconstant + Jt[0][1] * Fconstant)
    tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1])) - kv[1] * dot_theta[1] + ki[1] * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1]) + (Jt[1][0] * Fconstant + Jt[1][1] * Fconstant)

    tau3 = 0
    tau4 = kp[3] * (Jt[3][0] * (Xd[0] - X[0]) + Jt[3][1] * (Xd[1] - X[1])) - kv[3] * dot_theta[3] + ki[3] * (Jt[3][0] * sum_X[0] + Jt[3][1] * sum_X[1]) - (Jt[3][0] * Fconstant + Jt[3][1] * Fconstant)

    Tau = [tau1, tau2, tau3, tau4]

    return Tau


def new_PID_position_control_ver3(gain, dot_theta, Xd, X, Jt, sum_X, constantF=0):
    kp = []
    kv = []
    ki = []

    for i in range(len(gain)):
        Kp, Kv, Ki = gain[i]
        kp.append(Kp)
        kv.append(Kv)
        ki.append(Ki)

    er_vector = normal_vector(Xd[0], Xd[1])
    Jdt = desired_jacobi(Jt, er_vector)

    tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1])) - kv[0] * dot_theta[0] + ki[0] * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1]) +  constantF * Jdt[0]
    tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1])) - kv[1] * dot_theta[1] + ki[1] * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1]) +  constantF * Jdt[1]

    tau3 = 0
    tau4 = -constantF*Jdt[3]

    Tau = [tau1, tau2, tau3, tau4]

    return Tau


def new_PID_position_control_ver4(time, dot_theta,  gain, Xd, X, Jt, sum_X, constantF=0):
    kp = []
    kv = []
    ki = []

    for i in range(len(gain)):
        Kp, Kv, Ki = gain[i]
        kp.append(Kp)
        kv.append(Kv)
        ki.append(Ki)

    er_vector = normal_vector(Xd[0], Xd[1])
    Jdt = desired_jacobi(Jt, er_vector)

    tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1]))- kv[0] * dot_theta[0] + ki[0] * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1])
    tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1]))- kv[1] * dot_theta[1] + ki[1] * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1])

    tau3 = 0
    tau4 = 0

    if time > 5:
        if int(time) % 3 == 0:
            tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1])) - kv[0] * dot_theta[0] + ki[0] * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1]) + constantF * Jdt[0]
            tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1])) - kv[1] * dot_theta[1] + ki[1] * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1]) + constantF * Jdt[1]

            tau3 = 0
            tau4 = -constantF*Jdt[3]

        else:
            tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1])) - kv[0] * dot_theta[0] + ki[0] * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1]) - constantF * Jdt[0]
            tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1])) - kv[1] * dot_theta[1] + ki[1] * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1]) - constantF * Jdt[1]

            tau3 = 0
            tau4 = constantF*Jdt[3]

    Tau = [tau1, tau2, tau3, tau4]

    return Tau


def new_PID_position_control_ver5(gain, dot_theta, Xd, X, Jt, sum_X, constantF=0):
    kp = []
    kv = []
    ki = []

    for i in range(len(gain)):
        Kp, Kv, Ki = gain[i]
        kp.append(Kp)
        kv.append(Kv)
        ki.append(Ki)

    er_vector = normal_vector(Xd[0], Xd[1])
    Jdt = desired_jacobi(Jt, er_vector)

    tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1])) - kv[0] * dot_theta[0] + ki[0] * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1]) + constantF * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1]) * Jdt[0]
    tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1])) - kv[1] * dot_theta[1] + ki[1] * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1]) + constantF * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1]) * Jdt[1]

    tau3 = 0
    tau4 = -constantF*(Jt[3][0] * sum_X[0] + Jt[3][1] * sum_X[1])*Jdt[3]

    Tau = [tau1, tau2, tau3, tau4]

    return Tau


def new_PID_position_control_ver6(gain, dot_theta, Xd, X,
                                  Jt, sum_X, constantF=0):
    kp = []
    kv = []
    ki = []

    for i in range(len(gain)):
        Kp, Kv, Ki = gain[i]
        kp.append(Kp)
        kv.append(Kv)
        ki.append(Ki)

    er_vector = normal_vector(Xd[0], Xd[1])
    Jdt = desired_jacobi(Jt, er_vector)

    tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1])) - kv[0] * dot_theta[0] + ki[0] * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1]) + constantF * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1]) * Jdt[0]
    tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1])) - kv[1] * dot_theta[1] + ki[1] * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1]) + constantF * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1]) * Jdt[1]

    tau3 = 0
    tau4 = -constantF*(Jt[3][0] * sum_X[0] + Jt[3][1] * sum_X[1])*Jdt[3] - kv[3] * dot_theta[3]

    if X[0] < Xd[0] and X[1] > Xd[1]:
        er_vector = normal_vector(Xd[0], Xd[1])
        er_vector = [-1*er_vector[0], -1*er_vector[1]]
        Jdt = desired_jacobi(Jt, er_vector)
        tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1])) - kv[0] * dot_theta[0] + ki[0] * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1]) + constantF * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1]) * Jdt[0]
        tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1])) - kv[1] * dot_theta[1] + ki[1] * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1]) + constantF * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1]) * Jdt[1]

        tau3 = 0
        tau4 = -constantF*(Jt[3][0] * sum_X[0] + Jt[3][1] * sum_X[1])*Jdt[3] - kv[3] * dot_theta[3]

    Tau = [tau1, tau2, tau3, tau4]

    return Tau


def new_PID_position_control_ver7(gain, dot_theta, Xd, X, Jt, sum_X, constantF=0):
    kp = []
    kv = []
    ki = []

    for i in range(len(gain)):
        Kp, Kv, Ki = gain[i]
        kp.append(Kp)
        kv.append(Kv)
        ki.append(Ki)

    er_vector = normal_vector(Xd[0], Xd[1])
    Jdt = desired_jacobi(Jt, er_vector)

    tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1])) - kv[0] * dot_theta[0] + ki[0] * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1]) + constantF * (Jt[0][0] * sum_X[0] * er_vector[0] + Jt[0][1] * sum_X[1] * er_vector[1]) * Jdt[0]
    tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1])) - kv[1] * dot_theta[1] + ki[1] * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1]) + constantF * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1]) * Jdt[1]

    tau3 = 0
    tau4 = -constantF*(Jt[3][0] * sum_X[0]*er_vector[0] + Jt[3][1] * sum_X[1]*er_vector[1])*Jdt[3]

    Tau = [tau1, tau2, tau3, tau4]

    return Tau

def new_PID_position_control_ver8(gain, dot_theta, Xd, X, Jt, sum_X, constantF=0):
    kp = []
    kv = []
    ki = []

    for i in range(len(gain)):
        Kp, Kv, Ki = gain[i]
        kp.append(Kp)
        kv.append(Kv)
        ki.append(Ki)

    er_vector = normal_vector(Xd[0], Xd[1])
    Jdt = desired_jacobi(Jt, er_vector)

    tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1])) - kv[0] * dot_theta[0] + ki[0] * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1]) + constantF * (Jt[0][0] * sum_X[0] * er_vector[0] + Jt[0][1] * sum_X[1] * er_vector[1]) * Jdt[0]
    tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1])) - kv[1] * dot_theta[1] + ki[1] * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1]) + constantF * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1]) * Jdt[1]

    tau3 = 0
    tau4 = -constantF*(Jt[3][0] * (Xd[0] - X[0]) + Jt[3][1] * (Xd[1] - X[1]))*Jdt[3]

    Tau = [tau1, tau2, tau3, tau4]

    return Tau


def new_PID_position_control_ver9(gain, dot_theta, Xd, X, Jt, sum_X,
                                  constantF=0, eps=0.1):
    kp = []
    kv = []
    ki = []

    for i in range(len(gain)):
        Kp, Kv, Ki = gain[i]
        kp.append(Kp)
        kv.append(Kv)
        ki.append(Ki)

    norm = difference_norm(X[0], X[1], Xd[0], Xd[1])

    tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1])) + ki[0] * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1])
    tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1])) + ki[1] * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1])

    tau3 = 0
    tau4 = kp[3] * (Jt[3][0] * (Xd[0] - X[0]) + Jt[3][1] * (Xd[1] - X[1])) + ki[3] * (Jt[3][0] * sum_X[0] + Jt[3][1] * sum_X[1])

    f1, f2, f4 = 0, 0, 0

    if norm < eps:
        er_vector = unit_vector(X[0], X[1])
        Jdt = desired_jacobi(Jt, er_vector)

        f1 = constantF * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1]))
        f2 = constantF * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1]))
        f4 = -constantF * (Jt[3][0] * (Xd[0] - X[0]) + Jt[3][1] * (Xd[1] - X[1]))

        tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1])) - kv[0] * dot_theta[0] + ki[0] * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1]) + f1 * Jdt[0]
        tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1])) - kv[1] * dot_theta[1] + ki[1] * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1]) + f2 * Jdt[1]

        tau3 = 0
        tau4 = f4*Jdt[3]



    Tau = [tau1, tau2, tau3, tau4]

    f = [f1, f2, f4]

    return Tau, f


def new_PID_position_control_ver10(gain, dot_theta, Xd, X, Jt, sum_X,
                                  constantF=0, eps=0.1):
    kp = []
    kv = []
    ki = []

    for i in range(len(gain)):
        Kp, Kv, Ki = gain[i]
        kp.append(Kp)
        kv.append(Kv)
        ki.append(Ki)

    norm = difference_norm(X[0], X[1], Xd[0], Xd[1])

    tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1])) + ki[0] * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1])
    tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1])) + ki[1] * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1])

    tau3 = 0
    tau4 = kp[3] * (Jt[3][0] * (Xd[0] - X[0]) + Jt[3][1] * (Xd[1] - X[1])) + ki[3] * (Jt[3][0] * sum_X[0] + Jt[3][1] * sum_X[1])

    f1, f2, f4 = 0, 0, 0

    if norm < eps:
        er_vector = unit_vector(X[0], X[1])
        Jdt = desired_jacobi(Jt, er_vector)

        f1 = constantF
        f2 = constantF
        f4 = -constantF

        tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1])) - kv[0] * dot_theta[0] + ki[0] * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1]) + f1 * Jdt[0]
        tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1])) - kv[1] * dot_theta[1] + ki[1] * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1]) + f2 * Jdt[1]

        tau3 = 0
        tau4 = f4*Jdt[3]

    Tau = [tau1, tau2, tau3, tau4]
    f = [f1, f2, f4]

    return Tau, f


def new_PID_position_control_ver11(time, gain, dot_theta, Xd, X, Jt, sum_X, R,
                                   constantF=0, eps=0.1):
    kp = []
    kv = []
    ki = []

    for i in range(len(gain)):
        Kp, Kv, Ki = gain[i]
        kp.append(Kp)
        kv.append(Kv)
        ki.append(Ki)

    norm = difference_norm(X[0], X[1], Xd[0], Xd[1])
    er_vector = unit_vector(X[0], X[1])
    Jdt = desired_jacobi(Jt, er_vector)

    f_base = force_Normalization(norm, eps, constantF, R)
    if time == 0.0:
        f_base = 0.0

    f1 = f_base
    f2 = f_base
    f4 = -f_base


    tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1])) + ki[0] * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1]) + f1 * Jdt[0]
    tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1])) + ki[1] * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1]) + f2 * Jdt[1]

    tau3 = 0
    tau4 = kp[3] * (Jt[3][0] * (Xd[0] - X[0]) + Jt[3][1] * (Xd[1] - X[1])) + f4*Jdt[3]


    if norm < eps:

#        f1 = constantF * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1]))
#        f2 = constantF * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1]))
#        f4 = -constantF * (Jt[3][0] * (Xd[0] - X[0]) + Jt[3][1] * (Xd[1] - X[1]))

        f1 = constantF
        f2 = constantF
        f4 = -constantF


        tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1])) - kv[0] * dot_theta[0] + ki[0] * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1]) + f1 * Jdt[0]
        tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1])) - kv[1] * dot_theta[1] + ki[1] * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1]) + f2 * Jdt[1]

        tau3 = 0
        tau4 = f4*Jdt[3]

    Tau = [tau1, tau2, tau3, tau4]
    f = [f1, f2, f4]

    return Tau, f


def new_PID_position_control_ver12(gain, dot_theta, Xd, X, Jt, sum_X, R,
                                   constantF=0, eps=0.1):
    kp = []
    kv = []
    ki = []

    for i in range(len(gain)):
        Kp, Kv, Ki = gain[i]
        kp.append(Kp)
        kv.append(Kv)
        ki.append(Ki)

    norm = difference_norm(X[0], X[1], Xd[0], Xd[1])
    er_vector = unit_vector(X[0], X[1])
    Jdt = desired_jacobi(Jt, er_vector)

    f1 = 0
    f2 = 0
    f4 = 0

    tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1])) + ki[0] * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1]) + f1 * Jdt[0]
    tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1])) + ki[1] * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1]) + f2 * Jdt[1]

    tau3 = 0
    tau4 = kp[3] * (Jt[3][0] * (Xd[0] - X[0]) + Jt[3][1] * (Xd[1] - X[1])) + f4*Jdt[3]


    if norm <=  eps:

        f1 = constantF * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1]))
        f2 = constantF * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1]))
        f4 = -constantF * (Jt[3][0] * (Xd[0] - X[0]) + Jt[3][1] * (Xd[1] - X[1]))




        tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1])) - kv[0] * dot_theta[0] + ki[0] * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1]) + f1 * Jdt[0]
        tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1])) - kv[1] * dot_theta[1] + ki[1] * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1]) + f2 * Jdt[1]

        tau3 = 0
        tau4 = f4*Jdt[3]

    Tau = [tau1, tau2, tau3, tau4]
    f = [f1, f2, f4]

    return Tau, f


def positioncontorol_modified(gain, dot_theta, Xd, X, Jt, sum_X, F,
                              force_gain, ddot_eforce, dot_eforce, eforce,
                              sampling_time, constantF=0, eps=0.1):
    kp = []
    kv = []
    ki = []



    for i in range(len(gain)):
        Kp, Kv, Ki = gain[i]
        kp.append(Kp)
        kv.append(Kv)
        ki.append(Ki)

    norm = difference_norm(X[0], X[1], Xd[0], Xd[1])
    er_vector = unit_vector(X[0], X[1])
    Jdt = desired_jacobi(Jt, er_vector)

    f1 = 0
    f2 = 0
    f4 = 0

    tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1])) + ki[0] * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1]) + f1 * Jdt[0]
    tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1])) + ki[1] * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1]) + f2 * Jdt[1]

    tau3 = 0
    tau4 = kp[3] * (Jt[3][0] * (Xd[0] - X[0]) + Jt[3][1] * (Xd[1] - X[1])) + f4*Jdt[3]

    if norm <=  eps:

        fd1 = constantF * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1]))
        fd2 = constantF * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1]))
        fd4 = -constantF * (Jt[3][0] * (Xd[0] - X[0]) + Jt[3][1] * (Xd[1] - X[1]))


#        fd1 = constantF
#        fd2 = constantF
#        fd4 = -constantF


        ddot_eforce[0] = unit_P_control(force_gain, f1, fd1)
        ddot_eforce[1] = unit_P_control(force_gain, f2, fd2)
        ddot_eforce[2] = unit_P_control(force_gain, f4, fd4)
        eforce, dot_eforce, ddot_eforce = EulerMethod(eforce, dot_eforce,
                                                      ddot_eforce,
                                                      sampling_time)

        f1 = F[0] + ddot_eforce[0]
        f2 = F[1] + ddot_eforce[1]
        f4 = F[2] + ddot_eforce[2]


        tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1])) - kv[0] * dot_theta[0] + ki[0] * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1]) + f1 * Jdt[0]
        tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1])) - kv[1] * dot_theta[1] + ki[1] * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1]) + f2 * Jdt[1]

        tau3 = 0
        tau4 = f4*Jdt[3]

    Tau = [tau1, tau2, tau3, tau4]
    f = [f1, f2, f4]

    return Tau, f


def new_PID_position_control_4dof(gain, dot_theta, Xd, X, Jt, sum_X,
                                  constantF=0, eps=0.1, f0=0.0):
    kp = []
    kv = []
    ki = []

    for i in range(len(gain)):
        Kp, Kv, Ki = gain[i]
        kp.append(Kp)
        kv.append(Kv)
        ki.append(Ki)

    norm = difference_norm(X[0], X[1], Xd[0], Xd[1])

    tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1]))
    tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1]))
    tau3 = kp[2] * (Jt[2][0] * (Xd[0] - X[0]) + Jt[2][1] * (Xd[1] - X[1]))

    tau4 = 0
    tau5 = kp[4] * (Jt[4][0] * (Xd[0] - X[0]) + Jt[4][1] * (Xd[1] - X[1]))

    f1, f2, f3, f5 = 0, 0, 0, 0

    if norm < eps:
        er_vector = unit_vector(X[0], X[1])
        Jdt = desired_jacobi_4dof(Jt, er_vector)

        f1 = constantF * norm + f0
        f2 = constantF * norm + f0
        f3 = constantF * norm + f0
        f5 = -constantF * norm - f0

        tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1])) + f1*Jdt[0]
        tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1])) + f2*Jdt[1]
        tau3 = kp[2] * (Jt[2][0] * (Xd[0] - X[0]) + Jt[2][1] * (Xd[1] - X[1])) + f3*Jdt[2]

        tau4 = 0
        tau5 = f5*Jdt[4]

    Tau = [tau1, tau2, tau3, tau4, tau5]

    f = [f1, f2, f3, f5]

    return Tau, f

def new_PID_position_control_4dof_try(gain, dot_theta, Xd, X, Jt, sum_X,
                                      constantF=0, eps=0.1, f0=0.0):
    kp = []
    kv = []
    ki = []

    for i in range(len(gain)):
        Kp, Kv, Ki = gain[i]
        kp.append(Kp)
        kv.append(Kv)
        ki.append(Ki)

    norm = difference_norm(X[0], X[1], Xd[0], Xd[1])

    er_vector = unit_vector(X[0], X[1])
    Jdt = desired_jacobi_4dof(Jt, er_vector)

    f1 = -constantF * norm - f0
    f2 = -constantF * norm - f0
    f3 = -constantF * norm - f0
    f5 = constantF * norm + f0

    tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1])) + f1*Jdt[0]
    tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1])) + f2*Jdt[1]
    tau3 = kp[2] * (Jt[2][0] * (Xd[0] - X[0]) + Jt[2][1] * (Xd[1] - X[1])) + f3*Jdt[2]

    tau4 = 0
    tau5 = f5 * Jdt[4]

    Tau = [tau1, tau2, tau3, tau4, tau5]

    f = [f1, f2, f3, f5]

    return Tau, f


def PIDcontrol_eforce_base(gain1, gain2, theta, dot_theta, Xd, X, Jt, K, qd,
                           constantF=0, eps=0.1, f0=0.0):

    er_vector = normal_vector(X[0], X[1])
    Jdt = desired_jacobi_4dof(Jt, er_vector)

    norm = difference_norm(X[0], X[1], Xd[0], Xd[1])

    f1 = constantF * norm + f0
    f2 = constantF * norm + f0
    f3 = constantF * norm + f0
    f4 = -constantF * norm - f0
    f5 = -constantF * norm - f0

    tauf1 = f1 * Jdt[0]
    tauf2 = f2 * Jdt[1]
    tauf3 = f3 * Jdt[2]
    tauf4 = f4 * Jdt[3]
    tauf5 = f5 * Jdt[4]

    tauf = [tauf1, tauf2, tauf3, tauf4, tauf5]
    Thetad = deseired_theta(qd, tauf, K)


    kp1 = []
    kv1 = []
    ki1 = []

    kp2 = []
    kv2 = []
    ki2 = []

    for i in range(len(gain1)):
        Kp, Kv, Ki = gain1[i]
        kp1.append(Kp)
        kv1.append(Kv)
        ki1.append(Ki)

    for i in range(len(gain2)):
        Kp, Kv, Ki = gain2[i]
        kp2.append(Kp)
        kv2.append(Kv)
        ki2.append(Ki)

    kps = 0.01
    kvs = 0.001

    if norm > eps:
        tau1 = kp1[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1]))
        tau2 = kp1[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1]))
        tau3 = kp1[2] * (Jt[2][0] * (Xd[0] - X[0]) + Jt[2][1] * (Xd[1] - X[1]))

        tau4 = 0
        tau5 = kp1[4] * (Jt[4][0] * (Xd[0] - X[0]) + Jt[4][1] * (Xd[1] - X[1]))

        f1, f2, f3, f5 = 0, 0, 0, 0

    else:

        tau1 = kp2[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1])) + kps * (Thetad[0]-theta[0]) - kvs * dot_theta[0]
        tau2 = kp2[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1])) + kps * (Thetad[1]-theta[1]) - kvs * dot_theta[1]
        tau3 = kp2[2] * (Jt[2][0] * (Xd[0] - X[0]) + Jt[2][1] * (Xd[1] - X[1])) + kps * (Thetad[2]-theta[2]) - kvs * dot_theta[2]

        tau4 = 0
        tau5 = kps * (Thetad[4]-theta[4]) - kvs * dot_theta[4]

    Tau = [tau1, tau2, tau3, tau4, tau5]

    f = [f1, f2, f3, f5]

    return Tau, f, Thetad



def PIDcontrol_polar(gain1, gain2, theta, dot_theta, Xd, X, Jt, Jt_polar,
                     K, qd, Rd, R, Phid, Phi, Phi1, Phi2,
                     constantF=0, eps=0.1, f0=0.0):

    er_vector = normal_vector(X[0], X[1])
    Jdt = desired_jacobi_4dof(Jt, er_vector)

    norm = difference_norm(X[0], X[1], Xd[0], Xd[1])

    f1 = constantF * norm + f0
    f2 = constantF * norm + f0
    f3 = constantF * norm + f0
    f4 = -constantF * norm - f0
    f5 = -constantF * norm - f0

    tauf1 = f1 * Jt_polar[0][0]
    tauf2 = f2 * Jt_polar[1][0]
    tauf3 = f3 * Jt_polar[2][0]
    tauf4 = f4 * Jt_polar[3][0]
    tauf5 = f5 * Jt_polar[4][0]

    tauf = [tauf1, tauf2, tauf3, tauf4, tauf5]
    Thetad = deseired_theta(qd, tauf, K)


    kp1 = []
    kv1 = []
    ki1 = []

    kp2 = []
    kv2 = []
    ki2 = []

    for i in range(len(gain1)):
        Kp, Kv, Ki = gain1[i]
        kp1.append(Kp)
        kv1.append(Kv)
        ki1.append(Ki)

    for i in range(len(gain2)):
        Kp, Kv, Ki = gain2[i]
        kp2.append(Kp)
        kv2.append(Kv)
        ki2.append(Ki)

    kps = 0.01
    kvs = 0.001

    if norm > eps:
        tau1 = kp1[0] * (Jt_polar[0][0] * (Rd - R) + Jt_polar[0][1] * (Phid - Phi))
        tau2 = kp1[1] * (Jt_polar[1][0] * (Rd - R) + Jt_polar[1][1] * (Phid - Phi))
        tau3 = kp1[2] * (Jt_polar[2][0] * (Rd - R) + Jt_polar[2][1] * (Phid - Phi))

        tau4 = 0
        tau5 = kp1[4] * (Jt_polar[4][0] * (Rd - R) + Jt_polar[4][1] * (Phid - Phi))

        f1, f2, f3, f5 = 0, 0, 0, 0

    else:

        tau1 = kp2[0] * (Jt_polar[0][0] * (Rd - R) + Jt_polar[0][1] * (Phid - Phi)) + kps * (Thetad[0]-theta[0]) - kvs * dot_theta[0]
        tau2 = kp2[1] * (Jt_polar[1][0] * (Rd - R) + Jt_polar[1][1] * (Phid - Phi)) + kps * (Thetad[1]-theta[1]) - kvs * dot_theta[1]
        tau3 = kp2[2] * (Jt_polar[2][0] * (Rd - R) + Jt_polar[2][1] * (Phid - Phi)) + kps * (Thetad[2]-theta[2]) - kvs * dot_theta[2]

        tau4 = 0
        tau5 = kps * (Thetad[4]-theta[4]) - kvs * dot_theta[4]

    Tau = [tau1, tau2, tau3, tau4, tau5]

    f = [f1, f2, f3, f5]

    return Tau, f, Thetad




def new_PID_position_control_4dof_PID(gain, dot_theta, Xd, X, Jt, sum_X,
                                      constantF=0, eps=0.1):
    kp = []
    kv = []
    ki = []

    for i in range(len(gain)):
        Kp, Kv, Ki = gain[i]
        kp.append(Kp)
        kv.append(Kv)
        ki.append(Ki)

    norm = difference_norm(X[0], X[1], Xd[0], Xd[1])

    tau1 = kp[0] * (Jt[0][0] * (Xd[0] - X[0]) + Jt[0][1] * (Xd[1] - X[1])) + ki[0] * (Jt[0][0] * sum_X[0] + Jt[0][1] * sum_X[1])
    tau2 = kp[1] * (Jt[1][0] * (Xd[0] - X[0]) + Jt[1][1] * (Xd[1] - X[1])) + ki[1] * (Jt[1][0] * sum_X[0] + Jt[1][1] * sum_X[1])
    tau3 = kp[2] * (Jt[2][0] * (Xd[0] - X[0]) + Jt[2][1] * (Xd[1] - X[1])) + ki[2] * (Jt[2][0] * sum_X[0] + Jt[2][1] * sum_X[1])

    tau4 = 0
    tau5 = kp[4] * (Jt[4][0] * (Xd[0] - X[0]) + Jt[4][1] * (Xd[1] - X[1])) + ki[4] * (Jt[4][0] * sum_X[0] + Jt[4][1] * sum_X[1])

    f1, f2, f3, f5 = 0, 0, 0, 0

    Tau = [tau1, tau2, tau3, tau4, tau5]

    f = [f1, f2, f3, f5]

    return Tau, f


def normal_vector(Xd, Yd):
    z = pow(Xd, 2) + pow(Yd, 2)
    Absolute_value = sqrt(z)
    vector = [Xd/Absolute_value, Yd/Absolute_value]

    return vector


def unit_vector(x, y):
    z = pow(x, 2) + pow(y, 2)
    Absolute_value = sqrt(z)
    vector = [x/Absolute_value, y/Absolute_value]

    return vector


def difference_norm(x, y, xd, yd):
    squaring_norm = pow(xd-x, 2) + pow(yd-y, 2)
    norm = sqrt(squaring_norm)

    return norm


def deseired_theta(qd, tauf, K):
    Thetad = []

    for i in range(len(tauf)):
        a1 = 9 * tauf[i] * pow(K[i][1], 2) + sqrt(3) * sqrt(4 * pow(K[i][0], 3) * pow(K[i][1], 3) + 27 * pow(tauf[i], 2) * pow(K[i][1], 4))
        A1 = pow(a1, 1/3)
        A2 = pow(2, 1/3)*pow(3, 2/3) * K[i][1]

        thetad = qd[i] - (pow(2/3, 1/3) * K[i][0] / A1) + (A1 / A2)
        Thetad.append(thetad)

    return Thetad

def force_Normalization(norm, eps, constantf, R):
    a = 1
    x = norm - eps
    X = a / x
    maxX = 500
    minX = (1/(R[0]+R[1]-eps))

    f = constantf * (X - minX)/ (maxX- minX)

    if X >= maxX:
        f = constantf

    return f


def desired_jacobi(Jt, normal_vector):
    Jdt1 = (Jt[0][0] * normal_vector[0] + Jt[0][1] * normal_vector[1])
    Jdt2 = (Jt[1][0] * normal_vector[0] + Jt[1][1] * normal_vector[1])
    Jdt3 = (Jt[2][0] * normal_vector[0] + Jt[2][1] * normal_vector[1])
    Jdt4 = (Jt[3][0] * normal_vector[0] + Jt[3][1] * normal_vector[1])

    Jdt = [Jdt1, Jdt2, Jdt3, Jdt4]

    return Jdt


def desired_jacobi_4dof(Jt, normal_vector):
    Jdt1 = (Jt[0][0] * normal_vector[0] + Jt[0][1] * normal_vector[1])
    Jdt2 = (Jt[1][0] * normal_vector[0] + Jt[1][1] * normal_vector[1])
    Jdt3 = (Jt[2][0] * normal_vector[0] + Jt[2][1] * normal_vector[1])
    Jdt4 = (Jt[3][0] * normal_vector[0] + Jt[3][1] * normal_vector[1])
    Jdt5 = (Jt[4][0] * normal_vector[0] + Jt[4][1] * normal_vector[1])
    Jdt = [Jdt1, Jdt2, Jdt3, Jdt4, Jdt5]

    return Jdt


def moment_inertia(m, l):
    moment = m*l*l/12

    return moment


def link_inertia(m, l, Inertia):
    for i in range(len(m)):
        inertia = moment_inertia(m[i], l[i])
        Inertia.append(inertia)

    return Inertia


def sum_angle_difference(sum_angle, qd, q, sampling_time):
    for i in range(len(sum_angle)):
        for i in range(len(q)):
            sum_angle[i] += (qd[i]-q[i]) * sampling_time

    return sum_angle


def sum_position_difference(sum_x, xd, x, sampling_time):
        output_sum_x = []
        output_sum_x = sum_x + (xd - x)*sampling_time

        return output_sum_x


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


def angular_acceleration_4dof(inv_phi, f, A):
    ddot_q1 = inv_phi[0][0] * f[0] + inv_phi[0][1] * f[1] + inv_phi[0][2] * f[2] + inv_phi[0][3] * f[3] + inv_phi[0][4] * f[4] + inv_phi[0][5] * A[0] + inv_phi[0][6] * A[1]
    ddot_q2 = inv_phi[1][0] * f[0] + inv_phi[1][1] * f[1] + inv_phi[1][2] * f[2] + inv_phi[1][3] * f[3] + inv_phi[1][4] * f[4] + inv_phi[1][5] * A[0] + inv_phi[1][6] * A[1]
    ddot_q3 = inv_phi[2][0] * f[0] + inv_phi[2][1] * f[1] + inv_phi[2][2] * f[2] + inv_phi[2][3] * f[3] + inv_phi[2][4] * f[4] + inv_phi[2][5] * A[0] + inv_phi[2][6] * A[1]
    ddot_q4 = inv_phi[3][0] * f[0] + inv_phi[3][1] * f[1] + inv_phi[3][2] * f[2] + inv_phi[3][3] * f[3] + inv_phi[3][4] * f[4] + inv_phi[3][5] * A[0] + inv_phi[3][6] * A[1]
    ddot_q5 = inv_phi[4][0] * f[0] + inv_phi[4][1] * f[1] + inv_phi[4][2] * f[2] + inv_phi[4][3] * f[3] + inv_phi[4][4] * f[4] + inv_phi[4][5] * A[0] + inv_phi[4][6] * A[1]

    ddot_q = [ddot_q1, ddot_q2, ddot_q3, ddot_q4, ddot_q5]

    return ddot_q

def motor_angular_acceleration(Mm, tau, B, dot_theta, F):
    ddot_theta_data = []
    for i in range(len(tau)):
        ddot_theta = (tau[i] - B*dot_theta[i] - F[i])/Mm
        ddot_theta_data.append(ddot_theta)

    return ddot_theta_data


def EulerMethod(q, dot_q, ddot_q, sampling_time):
    for i in range(len(q)):
        dot_q[i] += ddot_q[i] * sampling_time
        q[i] += dot_q[i] * sampling_time

    return q, dot_q, ddot_q


def moment_matrix_3dof(m, l, lg, I, q):
    M1 = m[0] * lg[0] * lg[0] + I[0] + m[1] * (l[0] * l[0] + lg[1] * lg[1] + 2.0 * l[0] * lg[1] * cos(q[1])) + I[1]
    M2 = m[1] * (lg[1] * lg[1] + l[0] * lg[1] * cos(q[1])) + I[1]
    M3 = m[1] * (lg[1] * lg[1] + l[0] * lg[1] * cos(q[1])) + I[1]
    M4 = m[1] * lg[1] * lg[1] + I[1]

    M5 = m[2] * lg[2] * lg[2] + I[2] + m[3] * (l[2] * l[2] + lg[3] * lg[3] + 2.0 * l[2] * lg[3] * cos(q[2])) + I[3]
    M6 = m[3] * (lg[3] * lg[3] + l[2] * lg[3] * cos(q[3])) + I[3]
    M7 = m[3] * (lg[3] * lg[3] + l[2] * lg[3] * cos(q[3])) + I[3]
    M8 = m[3] * lg[3] * lg[3] + I[3]

    moment_matrix = [M1, M2, M3, M4, M5, M6, M7, M8]

    return moment_matrix


def moment_matrix_serial3dof(m, l, lg, I, q):
    M1 = m[0] * lg[0] * lg[0] + I[0] + m[1] * (l[0] * l[0] + lg[1] * lg[1] + 2.0*l[0] * lg[1] * cos(q[1])) + I[1] + m[2] * (l[0]*l[0] + l[1]*l[1] + lg[2]*lg[2] + 2*l[0]*l[1]*cos(q[1]) + 2*l[1]*lg[2]*cos(q[2]) + 2*l[0]*lg[2]*cos(q[1]+q[2])) + I[2]
    M2 = m[1] * (lg[1] * lg[1] + l[0] * lg[1] * cos(q[1])) + I[1]+ m[2]*(l[1] * l[1] + lg[2] * lg[2] + l[0] * l[1] * cos(q[1]) + 2*l[1]*lg[2]*cos(q[2]) + lg[0]*lg[2]*cos(q[1]+q[2])) + I[2]
    M3 = m[2]*(lg[2]*lg[2] + l[1]*lg[2]*cos(q[2]) + l[0]*lg[2]*cos(q[1]+q[2])) + I[2]

    M4 = m[1] * (lg[1] * lg[1] + l[0] * lg[1] * cos(q[1])) + I[1] + m[2]*(l[1]*l[1] + lg[2]*lg[2] + l[0]*l[1]*cos(q[1]) + 2*l[1]*lg[2]*cos(q[2]) + l[0]*lg[2]*cos(q[1] + q[2])) + I[2]
    M5 = m[1] * lg[1] * lg[1] + I[1] + m[2]*(l[1]*l[1] + lg[2]*lg[2]+2*l[1]*lg[2]*cos(q[2])) + I[2]
    M6 = m[2]*(lg[2]*lg[2] + l[1]*lg[2]*cos(q[2])) + I[2]

    M7 = m[2]*(lg[2]*lg[2] + l[1]*lg[2]*cos(q[2]) + l[0]*lg[2]*cos(q[1]+q[2])) + I[2]
    M8 = m[2]*(lg[2]*lg[2] + lg[1]*lg[2]*cos(q[2])) + I[2]
    M9 = m[2]*lg[2]*lg[2] + I[2]
    moment_matrix = [M1, M2, M3, M4, M5, M6, M7, M8, M9]

    return moment_matrix


def coriolis_item_serial3dof(m, l, lg, q, dot_q):
    h1 = -m[1]*l[0]*lg[1]*sin(q[1])*(dot_q[1]*dot_q[1] + 2.0*dot_q[0]*dot_q[1]) + m[2]*(-l[0] * l[1]  * dot_q[1] * sin(q[1]) * (2 * dot_q[0] + dot_q[1]) - l[1] * lg[2] * dot_q[2] * sin(q[2]) * (2 * dot_q[0] + 2 * dot_q[1] + dot_q[2]) - l[0] * lg[2] * (dot_q[1] + dot_q[2]) * sin(q[1] + q[2])  * (2 * dot_q[0] + dot_q[1] + dot_q[2]))
    h2 = m[1]*l[0]*lg[1]*sin(q[1])*dot_q[0]*dot_q[0] + m[2]*(l[0] * l[1]  * dot_q[0]  * dot_q[0] *  sin(q[1] - l[1] * lg[2] * sin(q[2]) * dot_q[2] * (2*dot_q[0]+2*dot_q[1]+dot_q[2]) + l[0] * lg[2] * dot_q[0]  * dot_q[0] * sin(q[1]+q[2])))
    h3 = m[2] * (l[1]*lg[2]*(dot_q[0]+dot_q[1])*(dot_q[0]+dot_q[1])*sin(q[2]) + l[0]*lg[2]*dot_q[0]*dot_q[0]*sin(q[1]+q[2]))

    H = [h1, h2, h3]

    return H

def moment_matrix_serial2dof(m, l, lg, I, q):
    M1 = m[0] * lg[0] * lg[0] + I[0] + m[1] * (l[0] * l[0] + lg[1] * lg[1] + 2.0 * l[0] * lg[1] * cos(q[1])) + I[1]
    M2 = m[1] * (lg[1] * lg[1] + l[0] * lg[1] * cos(q[1])) + I[1]
    M3 = m[1] * (lg[1] * lg[1] + l[0] * lg[1] * cos(q[1])) + I[1]
    M4 = m[1] * lg[1] * lg[1] + I[1]

    M = [M1, M2, M3, M4]

    return M

def coriolis_item_serial2dof(m, l, lg, q, dot_q):
    h1 = -m[1] * l[0] * lg[1] * (2 * dot_q[0] + dot_q[1]) * dot_q[1] * sin(q[1])
    h2 = m[1] * l[0] * lg[1] * dot_q[0] * dot_q[0] * sin(q[1])

    H = [h1, h2]

    return H


def define_E(l, q):
    E1 = -l[0] * sin(q[0]) - (l[1] * sin(q[0] + q[1]) + l[2] * sin(q[0] + q[1] + q[2]))
    E2 = -(l[1] * sin(q[0] + q[1]) + l[2] * sin(q[0] + q[1] + q[2]))
    E3 = -l[2] * sin(q[0] + q[1] + q[2])
    E4 = l[3] * sin(q[3]) + l[4] * sin(q[3] + q[4])
    E5 = l[4] * sin(q[3] + q[4])

    E6 = l[0] * cos(q[0]) + l[1] * cos(q[0] + q[1]) + l[2] * cos(q[0] + q[1] + q[2])
    E7 = l[1] * cos(q[0] + q[1]) + l[2] * cos(q[0] + q[1] + q[2])
    E8 = l[2] * cos(q[0] + q[1] + q[2])
    E9 = -l[3] * cos(q[3]) -l[4] * cos(q[3] + q[4])
    E10 = -l[4] * cos(q[3] + q[4])

    E = [E1, E2, E3, E4, E5, E6, E7, E8, E9, E10]

    return E


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

def phi_matrix_4dof(M1, M2, E):
    phi1, phi2, phi3, phi4, phi5, phi6, phi7 = M1[0], M1[1], M1[2], 0, 0, E[0], E[5]
    phi8, phi9, phi10, phi11, phi12, phi13, phi14 = M1[3], M1[4], M1[5], 0, 0, E[1], E[6]
    phi15, phi16, phi17, phi18, phi19, phi20, phi21 = M1[6], M1[7], M1[8], 0, 0, E[2], E[7]
    phi22, phi23, phi24, phi25, phi26, phi27, phi28 = 0, 0, 0, M2[0], M2[1], E[3], E[8]
    phi29, phi30, phi31, phi32, phi33, phi34, phi35 = 0, 0, 0, M2[2], M2[3], E[4], E[9]
    phi36, phi37, phi38, phi39, phi40, phi41, phi42 = E[0], E[1], E[2], E[3], E[4], 0, 0
    phi43, phi44, phi45, phi46, phi47, phi48, phi49 = E[5], E[6], E[7], E[8], E[9], 0, 0

    Phi = [[phi1, phi2, phi3, phi4, phi5, phi6, phi7],
           [phi8, phi9, phi10, phi11, phi12, phi13, phi14],
           [phi15, phi16, phi17, phi18, phi19, phi20, phi21],
           [phi22, phi23, phi24, phi25, phi26, phi27, phi28],
           [phi29, phi30, phi31, phi32, phi33, phi34, phi35],
           [phi36, phi37, phi38, phi39, phi40, phi41, phi42],
           [phi43, phi44, phi45, phi46, phi47, phi48, phi49]]

    return Phi


def phi_matrix(M, E):
    phi1, phi2, phi3, phi4, phi5, phi6 = M[0], M[1], 0, 0, E[0], E[4]
    phi7, phi8, phi9, phi10, phi11, phi12 = M[2], M[3], 0, 0, E[1], E[5]
    phi13, phi14, phi15, phi16, phi17, phi18 = 0, 0, M[4], M[5], E[2], E[6]
    phi19, phi20, phi21, phi22, phi23, phi24 = 0, 0, M[6], M[7], E[3], E[7]
    phi25, phi26, phi27, phi28, phi29, phi30 = E[0], E[1], E[2], E[3], 0, 0
    phi31, phi32, phi33, phi34, phi35, phi36 = E[4], E[5], E[6], E[7], 0, 0

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
        f = tau[i] - h[i] - B * dot_q[i]
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
        e = (theta[i] - q[i])
        e_data.append(e)

    return e_data


def non_linear_item(k, e):
    K = []
    for i in range(len(e)):
        kpart1 = k[i][0] + k[i][1]*pow(e[i], 2)
        kpart2 = kpart1*e[i]
        K.append(kpart2)

    return K


def restraint_part(l, q, dot_q, d):
    dot_P = -l[0] * sin(q[0]) * dot_q[0] - l[1] * sin(q[0] + q[1]) * (dot_q[0] + dot_q[1]) + l[2] * sin(q[2]) * dot_q[2] + l[3] * sin(q[2] + q[3]) * (dot_q[2] + dot_q[3])
    P = l[0] * cos(q[0]) + l[1] * cos(q[0] + q[1]) - l[2] * cos(q[2]) - l[3] * cos(q[2] + q[3]) - d[0]

    dot_Q = l[0] * cos(q[0]) * dot_q[0] + l[1] * cos(q[0] + q[1]) * (dot_q[0] + dot_q[1]) - l[2] * cos(q[2]) * dot_q[2] - l[3] * cos(q[2] + q[3]) * (dot_q[2] + dot_q[3])
    Q = l[0] * sin(q[0]) + l[1] * sin(q[0] + q[1]) - l[2] * sin(q[2]) - l[3] * sin(q[2] + q[3]) - d[1]

    return dot_P, P, dot_Q, Q



def restraint_item(l, q, dot_q):
    dot_P = -l[0] * sin(q[0]) * dot_q[0] - l[1] * sin(q[0] + q[1]) * (dot_q[0] + dot_q[1]) - l[2] * sin(q[0]+q[1]+q[2]) * (dot_q[0] + dot_q[1] + dot_q[2]) + l[3] * sin(q[3]) * dot_q[3] + l[4] * sin(q[3] + q[4]) * (dot_q[3] + dot_q[4])
    P = l[0] * cos(q[0]) + l[1] * cos(q[0] + q[1]) + l[2] * cos(q[0]+q[1]+q[2]) - l[3] * cos(q[3])-l[4]*cos(q[3] + q[4])

    dot_Q = l[0] * cos(q[0]) * dot_q[0] + l[1] * cos(q[0] + q[1]) * (dot_q[0] + dot_q[1]) + l[2] * cos(q[0]+q[1] + q[2]) * (dot_q[0] + dot_q[1] + dot_q[2]) - l[3] * cos(q[3]) * dot_q[3] - l[4] * cos(q[3] + q[4]) * (dot_q[3] + dot_q[4])
    Q = l[0] * sin(q[0]) + l[1] * sin(q[0] + q[1]) + l[2] * sin(q[0]+q[1]+q[2]) - l[3] * sin(q[3]) - l[4]*sin(q[3] + q[4])

    return dot_P, P, dot_Q, Q


def input_forces(l, q, dot_q, h, D, K, Jt,
                 P, Q, dot_P, dot_Q, Fx=0, Fy=0, s=1):
    f1 = K[0] + (Jt[0][0] * Fx + Jt[0][1] * Fy) - h[0] - D * dot_q[0]
    f2 = K[1] + (Jt[1][0] * Fx + Jt[1][1] * Fy) - h[1] - D * dot_q[1]
    f3 = K[2] + (Jt[2][0] * Fx + Jt[2][1] * Fy) - h[2] - D * dot_q[2]
    f4 = K[3] + (Jt[3][0] * Fx + Jt[3][1] * Fy) - h[3] - D * dot_q[3]

    A1 = -2 * s * dot_P - s * s * P + l[0] * cos(q[0]) * dot_q[0] * dot_q[0] + l[1] * cos(q[0] + q[1]) * (dot_q[0] + dot_q[1]) * (dot_q[0] + dot_q[1]) - l[2] * cos(q[2]) * dot_q[2] * dot_q[2] - l[3] * cos(q[2] + q[3]) * (dot_q[2] + dot_q[3]) * (dot_q[2] + dot_q[3])
    A2 = -2 * s * dot_Q - s * s * Q + l[0] * sin(q[0]) * dot_q[0] * dot_q[0] + l[1] * sin(q[0] + q[1]) * (dot_q[0] + dot_q[1]) * (dot_q[0] + dot_q[1]) - l[2] * sin(q[2]) * dot_q[2] * dot_q[2] - l[3] * sin(q[2] + q[3]) * (dot_q[2] + dot_q[3]) * (dot_q[2] + dot_q[3])

    f = [f1, f2, f3, f4]
    A = [A1, A2]

    return f, A


def input_forces_4dof(l, q, dot_q, h, D, K, Jt,
                      P, Q, dot_P, dot_Q, Fx=0, Fy=0, s=10):
    f1 = K[0] + (Jt[0][0] * Fx + Jt[0][1] * Fy) - h[0][0] - D * dot_q[0]
    f2 = K[1] + (Jt[1][0] * Fx + Jt[1][1] * Fy) - h[0][1] - D * dot_q[1]
    f3 = K[2] + (Jt[2][0] * Fx + Jt[2][1] * Fy) - h[0][2] - D * dot_q[2]
    f4 = (Jt[3][0] * Fx + Jt[3][1] * Fy) - h[1][0] - D * dot_q[3]
    f5 = K[4] + (Jt[4][0] * Fx + Jt[4][1] * Fy) - h[1][1] - D * dot_q[4]

    A1 = -2 * s * dot_P - s * s * P + l[0] * cos(q[0]) * dot_q[0] * dot_q[0] + l[1] * cos(q[0] + q[1]) * (dot_q[0] + dot_q[1]) * (dot_q[0] + dot_q[1]) + l[2] * cos(q[0] + q[1] + q[2]) * (dot_q[0] + dot_q[1] + dot_q[2]) * (dot_q[0] + dot_q[1] + dot_q[2]) - l[3] * cos(q[3]) * dot_q[3] * dot_q[3] - l[4] * cos(q[3] + q[4]) * (dot_q[3] + dot_q[4]) * (dot_q[3] + dot_q[4])
    A2 = -2 * s * dot_Q - s * s * Q + l[0] * sin(q[0]) * dot_q[0] * dot_q[0] + l[1] * sin(q[0] + q[1]) * (dot_q[0] + dot_q[1]) * (dot_q[0] + dot_q[1]) + l[2] * sin(q[0] + q[1] + q[2]) * (dot_q[0] + dot_q[1] + dot_q[2]) * (dot_q[0] + dot_q[1] + dot_q[2]) - l[3] * sin(q[3]) * dot_q[3] * dot_q[3] - l[4] * sin(q[3] + q[4]) * (dot_q[3] + dot_q[4]) * (dot_q[3] + dot_q[4])

    f = [f1, f2, f3, f4, f5]
    A = [A1, A2]

    return f, A


def out_forces(x, max_force=10, bias=1):
    y = bias*x
    if y > max_force:
        y = max_force
        F = y

    else:
        F = y

    return F


def binding_force_4dof(inv_phi, f, A):
    lambda_x = inv_phi[5][0] * f[0] + inv_phi[5][1] * f[1] + inv_phi[5][2] * f[2] + inv_phi[5][3] * f[3] + inv_phi[5][4] * f[4] + inv_phi[5][5] * A[0] + inv_phi[5][6] * A[1]
    lambda_y = inv_phi[6][0] * f[0] + inv_phi[6][1] * f[1] + inv_phi[6][2] * f[2] + inv_phi[6][3] * f[3] + inv_phi[6][4] * f[4] + inv_phi[6][5] * A[0] + inv_phi[6][6] * A[1]

    lam = [lambda_x, lambda_y]

    return lam


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


def jacobi_polar_coordinates_3dof(l, q):
    a = pow(l[0], 2) + pow(l[1], 2) + pow(l[2], 2) + 2*l[0]*l[1]*cos(q[1]) + 2*l[1]*l[2]*cos(q[2]) + 2*l[0]*l[2]*cos(q[1]+q[2])
    A = sqrt(a)

    J1 = 0
    J2 = (-l[0] * (l[1]*sin(q[1]) + l[2]*sin(q[1]+q[2]))) / A
    J3 = (-l[2] * (l[1]*sin(q[2]) + l[0]*sin(q[1]+q[2]))) / A

    J4 = 1
    J5 = 1
    J6 = 1

    J = [[J1, J2, J3], [J4, J5, J6]]

    return J


def jacobi_polar_coordinates_2dof(l, q):
    J1 = 0
    J2 = -(l[0]*l[1]*sin(q[1])) / sqrt(pow(l[0], 2) + pow(l[1], 2) + 2*l[0]*l[1]*cos(q[1]))
    J3 = 1
    J4 = 1

    J = [[J1, J2], [J3, J4]]

    return J


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


def jacobi_serial3dof(l, q):
    J1 = -l[0] * sin(q[0]) -l[1] * sin(q[0]+q[1]) - l[2] * sin(q[0] + q[1] + q[2])
    J2 = -l[1] * sin(q[0] + q[1]) - l[2] * sin(q[0] + q[1] + q[2])
    J3 = -l[2] * sin(q[0] + q[1] + q[2])

    J4 = l[0] * cos(q[0]) + l[1] * cos(q[0]+q[1]) + l[2] * cos(q[0] + q[1] + q[2])
    J5 = l[1] * cos(q[0]+q[1]) + l[2] * cos(q[0] + q[1] + q[2])
    J6 = l[2] * cos(q[0] + q[1] + q[2])

    J = [[J1, J2, J3], [J4, J5, J6]]

    return J


def jacobi_serial2dof(l, q):
#    J1 = -l[0] * sin(q[0]) - l[1] * sin(q[0] + q[1])
    J1 = 0
    J2 = -l[1] * sin(q[0] + q[1])
#    J3 = l[0] * cos(q[0]) + l[1] * cos(q[0] + q[1])
    J3 = 0
    J4 = l[1] * cos(q[0] + q[1])

    J = [[J1, J2], [J3, J4]]

    return J


def jacobi(J1, J2):
    J = [[J1[0][0], J1[0][1], J1[0][2], J2[0][0], J2[0][1]],
         [J1[1][0], J1[1][1], J1[1][2], J2[1][0], J2[1][1]]]

    return J


def make_circle(r,t, xd, yd):
    cx = r*cos(t) + xd
    cy = r*sin(t) + yd
    C = [cx, cy]
    return C


def print_non_lineaar_Characteristics():
    k1 = [0.003, 0.003]
    k2 = [0.003, 0.003, 0.003]
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
