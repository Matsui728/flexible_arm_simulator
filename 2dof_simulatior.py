# -*- coding: utf-8 -*-
"""
Created on Thu Aug 31 15:53:58 2017

@author: kawalab
"""

import numpy as np
import matplotlib.pyplot as plt
from math import degrees, radians
import print_result as pr
import simlib as sl


if __name__ == '__main__':
    # Parameters
    m1, m2 = 5.0, 5.0  # 質量
    l1, l2 = 0.5, 0.5  # リンク長さ
    lg1, lg2 = l1/2, l2/2  # 重心位置
    g = 9.8  # 重力加速度

    # 慣性モーメント
    I1 = sl.moment_inertia(m1, l1)
    I2 = sl.moment_inertia(m2, l2)

    # ゲイン調整
    kp1, kp2 = 3500, 1000
    kv1, kv2 = 300, 50
    ki1, ki2 = 100, 70

    q1, q2 = 0.0, 0.0    # 初期角度
    dot_q1, dot_q2 = 0.0, 0.0
    ddot_q1, ddot_q2 = 0.0, 0.0
    sum_q1, sum_q2 = 0.0, 0.0

    qd1, qd2 = radians(30), radians(45)   # 目標角度
    dot_qd1, dot_qd2 = 0.0, 0.0

    count_time = 10      # シミュレート時間
    sampling_time = 0.0001  # サンプリングタイム

    time_log = []
    deg1 = []
    deg2 = []
    dot_deg1 = []
    dot_deg2 = []
    ddot_deg1 = []
    ddot_deg2 = []
    link2_log = []

    ST = int(sl.simulation_time(count_time, sampling_time))

    f = open('2dof_simulation.csv', 'w')

    Parameters = ['Time[s]', 'q1', 'q2', 'qd1', 'qd2',
                  'dot_q1', 'dot_q2', 'ddot_q1', 'ddot_q2']
    f.write('Time[s], q1, q2, qd1, qd2, dot_q1, dot_q2, ddot_q1, ddot_q2\n')
    row_data = []

    for i in range(ST):

        time = i*sampling_time
        q1, q2, dot_q1, dot_q2 = sl.EulerMethod(q1, q2, dot_q1, dot_q2,
                                                ddot_q1, ddot_q2,
                                                sampling_time)

        M1, M2, M3, M4 = sl.inertia_term_2dof(m1, m2, l1, l2, lg1, lg2,
                                              I1, I2, q2)
        Minv1, Minv2, Minv3, Minv4 = sl.invm_2dof(M1, M2, M3, M4)

        h1, h2 = sl.coriolis_item_2dof(m2, l1, lg2, q2, dot_q1, dot_q2)

        G1, G2 = sl.gravity_item_2dof(m1, m2, l1, lg1, lg2, q1, q2, g)

        tau1 = sl.PIDcontrol(kp1, kv1, ki1, qd1, q1, dot_qd1, dot_q1, sum_q1)
        tau2 = sl.PIDcontrol(kp2, kv2, ki2, qd2, q2, dot_qd2, dot_q2, sum_q2)

        ddot_q1 = sl.calculate_angular_acceleration(Minv1, Minv2, tau1, tau2,
                                                    h1, h2, G1, G2)

        ddot_q2 = sl.calculate_angular_acceleration(Minv3, Minv4, tau1, tau2,
                                                    h1, h2, G1, G2)

        sum_q1 += sl.sum_angle_difference(qd1, q1, sampling_time)
        sum_q2 += sl.sum_angle_difference(qd2, q2, sampling_time)

        row = "{}, {}, {}, {}, {}, {}, {}, {}, {}\n". format(time,
                                                             degrees(q1),
                                                             degrees(q2),
                                                             degrees(qd1),
                                                             degrees(qd2),
                                                             dot_q1, dot_q2,
                                                             ddot_q1, ddot_q2)

        time_log = sl.save_log(time, time_log)
        deg1 = sl.save_log(degrees(q1), deg1)
        deg2 = sl.save_log(degrees(q2), deg2)
        dot_deg1 = sl.save_log(dot_q1, dot_deg1)
        dot_deg2 = sl.save_log(dot_q2, dot_deg2)
        ddot_deg1 = sl.save_log(ddot_q1, ddot_deg1)
        ddot_deg2 = sl.save_log(ddot_q2, ddot_deg2)

        f.write(row)

    x_data = [time_log]
    y_data = [deg1, deg2]
    label_name = ['deg1', 'deg2']

    plt.figure(figsize=(7, 3))
    pr.print_graph("Link Angle", time_log, y_data,
                   label_name, "Time[s]", "Angle[deg]", num_plot_data=2)
    plt.show()

    f.close()
