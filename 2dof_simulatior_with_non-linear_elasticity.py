# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:42:51 2017

@author: kawalab
"""

import numpy as np
import matplotlib.pyplot as plt
from math import degrees, radians
import print_result as pr
import simlib as sl


if __name__ == '__main__':
    # Parameters
    m1, m2 = 1.0, 1.0  # 質量
    l1, l2 = 0.3, 0.3  # リンク長さ
    lg1, lg2 = l1/2, l2/2  # 重心位置
    D = 0.05  # リンク粘性
    g = 9.8  # 重力加速度

    # Prametas of motor
    Mm = 34.7*pow(10, -7)       # モータの慣性モーメント
    B = 0.0  # モータの粘性

    # 慣性モーメント
    I1 = sl.moment_inertia(m1, l1)
    I2 = sl.moment_inertia(m2, l2)

    # ゲイン調整
    kp1, kp2 = 0.5, 0.5
    kv1, kv2 = 0.06, 0.06
    ki1, ki2 = 0.07, 0.07

    # Link data
    q1, q2 = 0.0, 0.0    # 初期角度
    dot_q1, dot_q2 = 0.0, 0.0
    ddot_q1, ddot_q2 = 0.0, 0.0
    sum_q1, sum_q2 = 0.0, 0.0
    # Desired Parametas
    qd1, qd2 = radians(0), radians(0)   # 目標角度
    dot_qd1, dot_qd2 = 0.0, 0.0

    # Motor data
    theta1, theta2 = 0.0, 0.0    # 初期角度
    dot_theta1, dot_theta2 = 0.0, 0.0
    ddot_theta1, ddot_theta2 = 0.0, 0.0
    sum_theta1, sum_theta2 = 0.0, 0.0
    # Desired Parametas
    thetad1, thetad2 = radians(30), radians(60)   # 目標角度
    dot_thetad1, dot_thetad2 = 0.0, 0.0

    # Non linear character Parametas
    k1 = [0.0038, 0.01]
    k2 = [0.01, 0.02]

    count_time = 20      # シミュレート時間
    sampling_time = 0.0001  # サンプリングタイム

    time_log = []
    link1 = []
    link2 = []
    dot_deg1 = []
    dot_deg2 = []
    ddot_deg1 = []
    ddot_deg2 = []

    motor1 = []
    motor2 = []
    dot_mdeg1 = []
    dot_mdeg2 = []
    ddot_mdeg1 = []
    ddot_mdeg2 = []

    desired_angle1 = []
    desired_angle2 = []

    ST = int(sl.simulation_time(count_time, sampling_time))

    fl = open('2dof_simulation_link_data.csv', 'w')
    fm = open('2dof_simulation_motor_data.csv', 'w')

    fl.write('Time[s], q1, q2, qd1, qd2, dot_q1, dot_q2, ddot_q1, ddot_q2\n')
    fm.write('Time[s], theta1, theta2, thetad1, thetad2, dot_theta1, dot_theta2, ddot_theta1, ddot_theta2\n')

    for i in range(ST):

        time = i*sampling_time  # 時間の設定

        # オイラー法の定義
        q1, q2, dot_q1, dot_q2 = sl.EulerMethod(q1, q2, dot_q1, dot_q2,
                                                ddot_q1, ddot_q2,
                                                sampling_time)

        (theta1, theta2,
         dot_theta1, dot_theta2) = sl.EulerMethod(theta1, theta2,
                                                  dot_theta1, dot_theta2,
                                                  ddot_theta1, ddot_theta2,
                                                  sampling_time)

        # 慣性行列の定義
        M1, M2, M3, M4 = sl.inertia_term_2dof(m1, m2, l1, l2, lg1, lg2,
                                              I1, I2, q2)
        # 逆行列の掃き出し
        Minv1, Minv2, Minv3, Minv4 = sl.invm_2dof(M1, M2, M3, M4)

        # コリオリ項の定義
        h1, h2 = sl.coriolis_item_2dof(m2, l1, lg2, q2, dot_q1, dot_q2)

        # 重力項の定義
        # G1, G2 = sl.gravity_item_2dof(m1, m2, l1, lg1, lg2, q1, q2, g)
        G1, G2 = 0.0, 0.0
        # 入力
        tau1 = sl.PIDcontrol(kp1, kv1, ki1, thetad1, theta1,
                             dot_thetad1, dot_theta1, sum_theta1)
        tau2 = sl.PIDcontrol(kp2, kv2, ki2, thetad2, theta2,
                             dot_thetad2, dot_theta2, sum_theta2)

        # 偏差と非線形弾性特性値の計算
        e1 = sl.difference_part(theta1, q1)
        e2 = sl.difference_part(theta2, q2)
        e = [e1, e2]

        K1 = sl.non_linear_item(k1[0], k2[0], e[0])
        K2 = sl.non_linear_item(k1[1], k2[1], e[1])
        K1 = K1*e[0]
        K2 = K2*e[1]
        K = [K1, K2]

        # 各加速度の計算
        ddot_q1 = sl.calculate_angular_acceleration(Minv1, Minv2, K[0], K[1],
                                                    h1, h2, G1, G2, D, dot_q1)

        ddot_q2 = sl.calculate_angular_acceleration(Minv3, Minv4, K[0], K[1],
                                                    h1, h2, G1, G2, D, dot_q2)

        ddot_theta1 = sl.motor_angular_acceleration(Mm, tau1, B,
                                                    dot_theta1, K[0])
        ddot_theta2 = sl.motor_angular_acceleration(Mm, tau2, B,
                                                    dot_theta2, K[1])

        # 偏差積分値の計算
        sum_q1 = sl.sum_angle_difference(sum_q1, qd1, q1, sampling_time)
        sum_q2 = sl.sum_angle_difference(sum_q2, qd2, q2, sampling_time)

        sum_theta1 = sl.sum_angle_difference(sum_theta1, thetad1,
                                             theta1, sampling_time)
        sum_theta2 = sl.sum_angle_difference(sum_theta2, thetad2,
                                             theta2, sampling_time)

        l_data = "{}, {}, {}, {}, {}, {}, {}, {}, {}\n". format(time,
                                                                degrees(q1),
                                                                degrees(q2),
                                                                degrees(qd1),
                                                                degrees(qd2),
                                                                dot_q1, dot_q2,
                                                                ddot_q1, ddot_q2)

        m_data = "{}, {}, {}, {}, {}, {}, {}, {}, {}\n". format(time,
                                                                degrees(theta1),
                                                                degrees(theta2),
                                                                degrees(thetad1),
                                                                degrees(thetad2),
                                                                dot_theta1, dot_theta2,
                                                                ddot_theta1, ddot_theta2)

        time_log = sl.save_log(time, time_log)
        desired_angle1 = sl.save_log(degrees(thetad1), desired_angle1)
        desired_angle2 = sl.save_log(degrees(thetad2), desired_angle2)

        # Save Link data
        link1 = sl.save_log(degrees(q1), link1)
        link2 = sl.save_log(degrees(q2), link2)
        dot_deg1 = sl.save_log(dot_q1, dot_deg1)
        dot_deg2 = sl.save_log(dot_q2, dot_deg2)
        ddot_deg1 = sl.save_log(ddot_q1, ddot_deg1)
        ddot_deg2 = sl.save_log(ddot_q2, ddot_deg2)

        # Save Motor data
        motor1 = sl.save_log(degrees(theta1), motor1)
        motor2 = sl.save_log(degrees(theta2), motor2)
        dot_mdeg1 = sl.save_log(dot_theta1, dot_mdeg1)
        dot_mdeg2 = sl.save_log(dot_theta2, dot_mdeg2)
        ddot_mdeg1 = sl.save_log(ddot_theta1, ddot_mdeg1)
        ddot_mdeg2 = sl.save_log(ddot_theta2, ddot_mdeg2)

        fl.write(l_data)
        fm.write(m_data)

    # Print result
    x_data = [time_log]
    link_data = [link1, link2, desired_angle1, desired_angle2]
    label_name1 = ['Link1', 'Link2', 'Desired angle1', 'Desired angle2']

    motor_data = [motor1, motor2, desired_angle1, desired_angle2]
    label_name2 = ['Motor1', 'Motor2', 'Desired angle1', 'Desired angle2']

    plt.figure(figsize=(5, 5))
    plt. subplot(2, 1, 1)
    pr.print_graph("Link Angle", time_log, link_data,
                   label_name1, "Time[s]", "Angle[deg]", num_plot_data=4)

    plt.subplot(2, 1, 2)
    pr.print_graph("Motor Angle", time_log, motor_data,
                   label_name2, "Time[s]", "Angle[deg]", num_plot_data=4)

    plt.show()

    fl.close()
    fm.close()
