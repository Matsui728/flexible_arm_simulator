# -*- coding: utf-8 -*-
"""
Created on Tue Feb 27 18:11:00 2018

@author: kawalab
"""

import numpy as np
import matplotlib.pyplot as plt
import print_result as pr
import simlib as sl
import inverse_kinematics as ik
from tqdm import tqdm
from math import degrees, radians, sin, cos, atan2, sqrt
from random import uniform


if __name__ == '__main__':

    # 第一アーム駆動トルク
    tau11, tau12, tau13 = 10, 1, 1
    tau1 = np.array([tau11, tau12, tau13])

    # 第二アーム駆動トルク
    tau22 = 0
    tau2 = np.array([0, tau22])

    # ロボットパラメータ
    m = [2, 2, 1, 2, 2]  # 質量
    arm1_m = [m[0], m[1], m[2]]
    arm2_m = [m[3], m[4]]

    ll = [0.3, 0.3, 0.1, 0.35, 0.35]  # リンク長さ
    link1 = [ll[0], ll[1], ll[2]]
    link2 = [ll[3], ll[4]]
    L = [link1, link2]

    lg = [ll[0]/2, ll[1]/2, ll[2]/2, ll[3]/2, ll[4]/2]  # 重心位置
    arm1_lg = [lg[0], lg[1], lg[2]]
    arm2_lg = [lg[3], lg[4]]

    # 初期姿勢
    x0 = 0.0
    y0 = 0.4
    Theta = radians(0)

    q, theta = ik.make_intial_angle(x0, y0, link1, link2, Theta)  # 初期角度

    # Link data
    dot_q = [0.0, 0.0, 0.0, 0.0, 0.0]
    ddot_q = [0.0, 0.0, 0.0, 0.0, 0.0]
    sum_q = [0.0, 0.0, 0.0, 0.0, 0.0]

    arm1_q = [q[0], q[1], q[2]]
    arm2_q = [q[3], q[4]]

    arm1_dotq = [dot_q[0], dot_q[1], dot_q[2]]
    arm2_dotq = [dot_q[3], dot_q[4]]

    arm1_ddotq = [ddot_q[0], ddot_q[1], ddot_q[2]]
    arm2_ddotq = [ddot_q[3], ddot_q[4]]

    # Desired Parametas
    dot_qd = [0.0, 0.0, 0.0, 0.0, 0.0]

    # ヤコビ行列
    J1 = sl.jacobi_serial3dof(link1, arm1_q)
    J2 = sl.jacobi_serial2dof(link2, arm2_q)

    Jt1 = sl.transpose_matrix(J1)
    Jt2 = sl.transpose_matrix(J2)

    InvJ2 = sl.inverse_matrix(Jt2)

    J1_seudo = sl.pseudo_inverse_matrix(J1, Jt1)

    # 3*3単位行列
    eye = np.eye(3)

    f2 = Jt2.dot(tau2)

    X1 = sl.inverse_matrix(eye - Jt1.dot(J1_seudo))
    X2 = ((Jt1.dot(-f2)) - tau1)

    k = X1.dot(X2)

    N = (eye - Jt1.dot(J1_seudo))

    ttau1 = np.transpose(N.dot(k))

    value = J1_seudo.dot(ttau1)

    print("k1={}, k2={}, k3={}" . format(k[0], k[1], k[2]))

    print(value)

    print('tasks were compleated!')
