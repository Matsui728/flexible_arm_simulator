# -*- coding: utf-8 -*-
"""
Created on Sat Sep  9 16:32:10 2017

@author: kawalab
"""

import numpy as np
import matplotlib.pyplot as plt
from math import degrees, radians
import print_result as pr
import simlib as sl


if __name__ == '__main__':
    # Parameters
    m = [1.0, 1.0, 1.0, 1.0]  # 質量
    ll = [0.3, 0.3, 0.3, 0.3]  # リンク長さ
    lg = [ll[0]/2, ll[1]/2, ll[2]/2, ll[3]/2]  # 重心位置
    D = 0.05  # リンク粘性
    g = 9.8  # 重力加速度

    # Prametas of motor
    Mm = 34.7*pow(10, -7)       # モータの慣性モーメント
    B = 0.0  # モータの粘性

    # 慣性モーメント
    Inertia = []
    Inertia = sl.link_inertia(m, ll, Inertia)

    # ゲイン調整
    control_gain1 = sl.imput_gain(0.0, 0.0, 0.0)
    control_gain2 = sl.imput_gain(0.0, 0.0, 0.0)
    control_gain3 = sl.imput_gain(0.0, 0.0, 0.0)
    control_gain4 = sl.imput_gain(0.0, 0.0, 0.0)
    gain = [control_gain1, control_gain2, control_gain3, control_gain4]

    # Link data
    q = [0.0, 0.0, 0.0, 0.0]    # 初期角度
    dot_q = [0.0, 0.0, 0.0, 0.0]
    ddot_q = [0.0, 0.0, 0.0, 0.0]
    sum_q = [0.0, 0.0, 0.0, 0.0]
    # Desired Parametas
    qd = [radians(0), radians(0), radians(0), radians(0)]    # 目標角度
    dot_qd = [0.0, 0.0, 0.0, 0.0]

    # Motor data
    theta = [0.0, 0.0, 0.0, 0.0]    # 初期角度
    dot_theta = [0.0, 0.0, 0.0, 0.0]
    ddot_theta = [0.0, 0.0, 0.0, 0.0]
    sum_theta = [0.0, 0.0, 0.0, 0.0]
    # Desired Parametas
    thetad = [radians(0), radians(0), radians(0), radians(0)]    # 目標角度
    dot_thetad = [0.0, 0.0, 0.0, 0.0]

    # Non linear character Parametas
    k1 = sl.non_linear_parameta(0.001, 0.001)
    k2 = sl.non_linear_parameta(0.001, 0.001)
    k3 = sl.non_linear_parameta(0.001, 0.001)
    k4 = sl.non_linear_parameta(0.001, 0.001)

    k = [k1, k2, k3, k4]

    # Time Parametas
    simulate_time = 30      # シミュレート時間
    sampling_time = 0.001  # サンプリングタイム

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

    ST = int(sl.simulation_time(simulate_time, sampling_time))

    fl = open('2dof_simulation_link_data.csv', 'w')
    fm = open('2dof_simulation_motor_data.csv', 'w')

    fl.write('Time[s], q1, q2, qd1, qd2, dot_q1, dot_q2, ddot_q1, ddot_q2\n')
    fm.write('Time[s], theta1, theta2, thetad1, thetad2, dot_theta1, dot_theta2, ddot_theta1, ddot_theta2\n')

    for i in range(ST):

        time = i*sampling_time  # 時間の設定

        # オイラー法の定義
        q, dot_q, ddot_q = sl.EulerMethod(q, dot_q, ddot_q, sampling_time)

        theta, dot_theta, ddot_theta = sl.EulerMethod(theta, dot_theta,
                                                      ddot_theta,
                                                      sampling_time)

        # 慣性行列の定義
        mm = sl.moment_matrix_3dof(m, ll, lg, Inertia, q)

        # コリオリ項の定義
        H = sl.coriolis_item_3dof(m, ll, lg, Inertia, q, dot_q)

        # 重力項の定義
        g1, g2, g3, g4 = 0.0, 0.0, 0.0, 0.0
        G = [g1, g2, g3, g4]

        # 二回微分値の導出
        E = sl.twice_differential_values(ll, q)

        # 逆行列の掃き出し
        Phi = sl.phi_matrix(mm, E)
        invPhi = sl.inverse_matrix(Phi)

        # ヤコビとヤコビ転置
        J = sl.jacobi_matrix(ll, q)

        Jt = sl.transpose_matrix(J)

        # モータ入力
        Tau = sl.PID_angle_control(gain, qd, q, dot_qd, dot_q, sum_q)

        # 偏差と非線形弾性特性値の計算
        e = sl.difference_part(theta, q)

        K = sl.non_linear_item(k[:][0], k[:][1], e)

        # 拘束力とダイナミクス右辺の計算
        dot_P, P, dot_Q, Q = sl.restraint_part(ll, q, dot_q)

        f, A = sl.input_forces(ll, q, dot_q, H, D, K,
                               Jt, P, Q, dot_P, dot_Q, Fx=0, Fy=0, s=1)

        # 関節角加速度の計算
        ddot_q = sl.angular_acceleration_3dof(invPhi, f, A)

        # 拘束項
        lam = sl.binding_force(invPhi, f, A)

        # モータ角加速度の計算
        ddot_theta = sl.motor_angular_acceleration(Mm, Tau[i-1], B,
                                                   dot_theta[i-1], K)




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
