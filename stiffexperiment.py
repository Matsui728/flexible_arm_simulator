# -*- coding: utf-8 -*-
"""
Created on Tue Feb 20 15:30:23 2018

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

    tau22 = 1
    tau2 = np.array([0, tau22])
    tau11_data = []
    tau12_data = []
    r_data = []

    ll = [0.3, 0.3, 0.3, 0.3]  # リンク長さ
    link1 = [ll[0], ll[1]]
    link2 = [ll[2], ll[3]]
    L = [link1, link2]

    lg = [ll[0]/2, ll[1]/2, ll[2]/2, ll[3]/2]  # 重心位置
    arm1_lg = [lg[0], lg[1]]
    arm2_lg = [lg[2], lg[3]]

    # 初期姿勢

    r = 0

    for i in range(3000):
        r = -0.15 + 0.0001*i

        x10 = 0
        y10 = 0.3

        x20 = x10 + r
        y20 = y10

        arm1_q = ik.cul_inverse_kinematics_2dof(x10, y10, link1)
        arm2_q = ik.cul_inverse_kinematics_2dof(x20, y20, link2)


        arm1_q = [radians(arm1_q[0]), radians(arm1_q[1])]
        arm2_q = [radians(arm2_q[0]), radians(arm2_q[1])]

        # Link data
        dot_q = [0.0, 0.0, 0.0, 0.0]
        ddot_q = [0.0, 0.0, 0.0, 0.0]
        sum_q = [0.0, 0.0, 0.0, 0.0]

        arm1_dotq = [dot_q[0], dot_q[1]]
        arm2_dotq = [dot_q[2], dot_q[3]]

        arm1_ddotq = [ddot_q[0], ddot_q[1]]
        arm2_ddotq = [ddot_q[2], ddot_q[3]]

        # ヤコビとヤコビ転置
        J1 = sl.jacobi_serial2dof(link1, arm1_q)
        J2 = sl.jacobi_serial2dof(link2, arm2_q)

        Jt1 = sl.transpose_matrix(J1)
        Jt2 = sl.transpose_matrix(J2)

        Jt2I = sl.inverse_matrix(Jt2)

        f2 = Jt2I.dot(tau2)
        n = sl.unit_vector(-x20, -y20)

        f2_norm = sqrt(pow(f2[0], 2) + pow(f2[1], 2))
        nfx = n[0]*f2_norm
        nfy = n[1]*f2_norm
        nf = [nfx, nfy]

        tau1 = Jt1.dot(nf)

        tau11_data = pr.save_part_log(tau1[0], tau11_data)
        tau12_data = pr.save_part_log(tau1[1], tau12_data)

        r_data = pr.save_part_log(r, r_data)

        tau1_data = [tau11_data, tau12_data]


    label_name = ['tau11', 'tau12']
    plt.figure(figsize=(5, 5))

    pr.print_graph('tau1-rdata', r_data,
                         tau1_data, label_name,
                         'r[m]', 'Torque[Nm]',
                         num_plot_data=2)

    plt.savefig('2_2sim0303.eps')
    plt.show()
