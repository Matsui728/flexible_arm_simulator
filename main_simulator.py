# -*- coding: utf-8 -*-
"""
Created on Thu Aug 31 15:53:58 2017

@author: kawalab
"""

import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, degrees, radians

import csv


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


if __name__ == '__main__':
    # Parameters
    m1, m2 = 5.0, 5.0  # 質量
    l1, l2 = 0.5, 0.5  # リンク長さ
    lg1, lg2 = l1/2, l2/2  # 重心位置
    g = 9.8  # 重力加速度

    # 慣性モーメント
    I1 = moment_inertia(m1, l1)
    I2 = moment_inertia(m2, l2)

    # ゲイン調整
    kp1, kp2 = 3500, 1000
    kv1, kv2 = 300, 50
    ki1, ki2 = 100, 70

    q1, q2 = 0.0    # 初期角度
    dot_q1, dot_q2 = 0.0
    ddot_q1, ddot_q2 = 0.0
    sum_q1, sum_q2 = 0.0

    qd1, qd2 = radians(30), radians(45)   # 目標角度
    dot_qd1, dot_qd2 = 0.0

    count_time = 10      # シミュレート時間
    sampling_time = 0.0001  # サンプリングタイム

    link1_log = []
    link2_log = []

    ST = simulation_time(count_time, sampling_time)

    f = open('2dof_simulation.csv', 'ab')
    dataWriter = csv.writer(f)
    Parameters = ['Time[s]', 'q1', 'q2', 'qd1', 'qd2',
                  'dot_q1', 'dot_q2', 'ddot_q1', 'ddot_q2']
    dataWriter.writerow(Parameters)
    rowList = []

    for i in ST:

        time = i*sampling_time
        q1, q2, dot_q1, dot_q2 = EulerMethod(q1, q2, dot_q1, dot_q2,
                                             ddot_q1, ddot_q2, sampling_time)

        M1, M2, M3, M4 = inertia_term_2dof(m1, m2, l1, l2, lg1, lg2,
                                           I1, I2, q2)
        Minv1, Minv2, Minv3, Minv4 = invm_2dof(M1, M2, M3, M4)

        h1, h2 = coriolis_item_2dof(m2, l1, lg2, q2, dot_q1, dot_q2)

        G1, G2 = gravity_item_2dof(m1, m2, l1, lg1, lg2, q1, q2, g)

        tau1 = PIDcontrol(kp1, kv1, ki1, qd1, q1, dot_qd1, dot_q1, sum_q1)
        tau2 = PIDcontrol(kp2, kv2, ki2, qd2, q2, dot_qd2, dot_q2, sum_q2)

        ddot_q1 = calculate_angular_acceleration(Minv1, Minv2, tau1, tau2,
                                                 h1, h2, G1, G2)

        ddot_q2 = calculate_angular_acceleration(Minv3, Minv4, tau1, tau2,
                                                 h1, h2, G1, G2)

        sum_q1 += sum_angle_difference(qd1, q1, sampling_time)
        sum_q2 += sum_angle_difference(qd2, q2, sampling_time)

        rowList.append([])
        data_set = [time, degrees(q1), degrees(q2), degrees(qd1), degrees(qd2),
                    dot_q1, dot_q2, ddot_q1, ddot_q2]
        rowList.append(data_set)
        dataWriter.writerows(rowList)

    f.close()
