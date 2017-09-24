# -*- coding: utf-8 -*-
"""
Created on Sat Sep  9 16:32:10 2017

@author: kawalab
"""

import numpy as np
import matplotlib.pyplot as plt
from math import degrees, radians, sin, cos, sqrt
import print_result as pr
import simlib as sl
from tqdm import tqdm


if __name__ == '__main__':
    # Parameters
    m = [1.0, 1.0, 1.0, 1.0]  # 質量
    ll = [0.3, 0.3, 0.3, 0.3]  # リンク長さ
    lg = [ll[0]/2, ll[1]/2, ll[2]/2, ll[3]/2]  # 重心位置
    D = 0.1  # リンク粘性
    g = 9.8  # 重力加速度

    # Prametas of motor
    Mm = 34.7*pow(10, -7)       # モータの慣性モーメント
    B = 0.0  # モータの粘性

    # 慣性モーメント
    Inertia = []
    Inertia = sl.link_inertia(m, ll, Inertia)

    # ゲイン調整
    control_gain1 = sl.imput_gain(10.0, 0.004, 5.0)
    control_gain2 = sl.imput_gain(10.0, 0.004, 5.0)
    control_gain3 = sl.imput_gain(0.0, 0.0, 0.0)
    control_gain4 = sl.imput_gain(10.0, 0.004, 5.0)
    gain = [control_gain1, control_gain2, control_gain3, control_gain4]

    # Link data
    q = [radians(45), radians(90), radians(45), radians(90)]    # 初期角度
    dot_q = [0.0, 0.0, 0.0, 0.0]
    ddot_q = [0.0, 0.0, 0.0, 0.0]
    sum_q = [0.0, 0.0, 0.0, 0.0]

    # Desired Parametas
    qd = [radians(0), radians(0), radians(0), radians(0)]    # 目標角度
    dot_qd = [0.0, 0.0, 0.0, 0.0]

    # Motor data
    theta = [radians(45), radians(90), radians(45), radians(90)]    # 初期角度
    dot_theta = [0.0, 0.0, 0.0, 0.0]
    ddot_theta = [0.0, 0.0, 0.0, 0.0]
    sum_theta = [0.0, 0.0, 0.0, 0.0]
    # Desired Parametas
    thetad = [radians(0), radians(0), radians(0), radians(0)]    # 目標角度
    dot_thetad = [0.0, 0.0, 0.0, 0.0]

    # Data list
    (q1_data, q2_data, q3_data, q4_data) = [], [], [], []
    (qd1_data, qd2_data, qd3_data, qd4_data) = [], [], [], []
    (dot_q1_data, dot_q2_data, dot_q3_data, dot_q4_data) = [], [], [], []
    (ddot_q1_data, ddot_q2_data, ddot_q3_data, ddot_q4_data) = [], [], [], []
    (sum_q1_data, sum_q2_data, sum_q3_data, sum_q4_data) = [], [], [], []
    (dot_qd1_data, dot_qd2_data, dot_qd3_data, dot_qd4_data) = [], [], [], []
    (q_data, qd_data, dot_q_data,
     ddot_q_data, sum_q_data, dot_qd_data) = [], [], [], [], [], []

    (theta1_data, theta2_data, theta3_data, theta4_data) = [], [], [], []
    (thetad1_data, thetad2_data, thetad3_data, thetad4_data) = [], [], [], []
    (dot_theta1_data, dot_theta2_data,
     dot_theta3_data, dot_theta4_data) = [], [], [], []
    (ddot_theta1_data, ddot_theta2_data,
     ddot_theta3_data, ddot_theta4_data) = [], [], [], []
    (sum_theta1_data, sum_theta2_data,
     sum_theta3_data, sum_theta4_data) = [], [], [], []

    (theta_data, thetad_data, dot_theta_data,
     ddot_theta_data,
     sum_theta_data, dot_thetad_data) = [], [], [], [], [], []

    # Non linear character Parametas
    k1 = sl.non_linear_parameta(1.0, 1.0)
    k2 = sl.non_linear_parameta(1.0, 1.0)
    k3 = sl.non_linear_parameta(0.0, 0.0)
    k4 = sl.non_linear_parameta(1.0, 1.0)

    k = [k1, k2, k3, k4]

    # Time Parametas
    simulate_time = 5      # シミュレート時間
    sampling_time = 0.001  # サンプリングタイム

    # Deseired Position
    xd = -0.1
    yd = 0.4
    Xd = [xd, yd]
    x_data = []
    xd_data = []
    yd_data = []
    y_data = []
    c_data = []
    sum_x = 0.0
    sum_y = 0.0
    sum_x_data = []
    sum_y_data = []

    lamx_data = []
    lamy_data = []

    time_log = []

    ST = int(sl.simulation_time(simulate_time, sampling_time))

    fl = open('3dof_simulation_link_data.csv', 'w')
    fm = open('3dof_simulation_motor_data.csv', 'w')
    fp = open('3dof_simulation_position_data.csv', 'w')

    fl.write('Time[s], q1, q2, q3, q4, qd1, qd2, qd3, qd4,'
             + 'dot_q1, dot_q2, dot_q3, dot_q4,'
             + 'ddot_q1, ddot_q2, ddot_q3, ddot_q4\n')

    fm.write('Time[s], theta1, theta2, theta3, theta4,'
             + 'thetad1, thetad2, thetad3, thetad4,'
             + 'dot_theta1, dot_theta2, dot_theta3, dot_theta4,'
             + 'ddot_theta1, ddot_theta2, ddot_theta3, ddot_theta4\n')

    fp.write('Time[s], X, Y, Xd, Yd, lambdax, lambday\n')

    for i in tqdm(range(ST)):

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

        # 手先位置導出
        X = ll[0] * cos(q[0]) + ll[1] * cos(q[0] + q[1])
        Y = ll[0] * sin(q[0]) + ll[1] * sin(q[0] + q[1])

        position = [X, Y]

        # 偏差積分値の計算
        sum_x = sl.sum_position_difference(sum_x, xd, X, sampling_time)
        sum_y = sl.sum_position_difference(sum_y, yd, Y, sampling_time)

        sum_X = [sum_x, sum_y]

        # モータ入力
        Tau = sl.PID_potiton_control_3dof(gain, Xd, position,
                                          Jt, dot_theta, sum_X)

        # 偏差と非線形弾性特性値の計算
        e = sl.difference_part(theta, q)

        K = sl.non_linear_item(k, e)

        # 拘束力とダイナミクス右辺の計算
        dot_P, P, dot_Q, Q = sl.restraint_part(ll, q, dot_q)

        f, A = sl.input_forces(ll, q, dot_q, H, D, K,
                               Jt, P, Q, dot_P, dot_Q, Fx=0, Fy=0, s=1)

        # 関節角加速度の計算
        ddot_q = sl.angular_acceleration_3dof(invPhi, f, A)

        # 拘束項
        lam = sl.binding_force(invPhi, f, A)

        # モータ角加速度の計算
        ddot_theta = sl.motor_angular_acceleration(Mm, Tau, B,
                                                   dot_theta, K)

        # エクセル用log_data保存
        link_data = pr.save_angle_excel_log(time, q, qd, dot_q, ddot_q)
        motor_data = pr.save_angle_excel_log(time, theta, thetad,
                                             dot_theta, ddot_theta)

        position_data = pr.save_position_log(time, position[0], position[1],
                                             xd, yd, lam[0], lam[1])

        # Save time log
        time_log = pr.save_part_log(time, time_log)

        # Save link_data(radian)
        qd1_data = pr.save_part_log(degrees(qd[0]), qd1_data)
        qd2_data = pr.save_part_log(degrees(qd[1]), qd2_data)
        qd3_data = pr.save_part_log(degrees(qd[2]), qd3_data)
        qd4_data = pr.save_part_log(degrees(qd[3]), qd4_data)
        qd_data = [qd1_data, qd2_data, qd3_data, qd4_data]

        q1_data = pr.save_part_log(degrees(q[0]), q1_data)
        q2_data = pr.save_part_log(degrees(q[1]), q2_data)
        q3_data = pr.save_part_log(degrees(q[2]), q3_data)
        q4_data = pr.save_part_log(degrees(q[3]), q4_data)
        q_data = [q1_data, q2_data, q3_data, q4_data]

        dot_q1_data = pr.save_part_log(dot_q[0], dot_q1_data)
        dot_q2_data = pr.save_part_log(dot_q[1], dot_q2_data)
        dot_q3_data = pr.save_part_log(dot_q[2], dot_q3_data)
        dot_q4_data = pr.save_part_log(dot_q[3], dot_q4_data)
        dot_q_data = [dot_q1_data, dot_q2_data, dot_q3_data, dot_q4_data]

        ddot_q1_data = pr.save_part_log(ddot_q[0], ddot_q1_data)
        ddot_q2_data = pr.save_part_log(ddot_q[1], ddot_q2_data)
        ddot_q3_data = pr.save_part_log(ddot_q[2], ddot_q3_data)
        ddot_q4_data = pr.save_part_log(ddot_q[3], ddot_q4_data)
        ddot_q_data = [ddot_q1_data, ddot_q2_data, ddot_q3_data, ddot_q4_data]

        # Save Motor data(radian)
        thetad1_data = pr.save_part_log(degrees(thetad[0]), thetad1_data)
        thetad2_data = pr.save_part_log(degrees(thetad[1]), thetad2_data)
        thetad3_data = pr.save_part_log(degrees(thetad[2]), thetad3_data)
        thetad4_data = pr.save_part_log(degrees(thetad[3]), thetad4_data)
        thetad_data = [thetad1_data, thetad2_data, thetad3_data, thetad4_data]

        theta1_data = pr.save_part_log(degrees(theta[0]), theta1_data)
        theta2_data = pr.save_part_log(degrees(theta[1]), theta2_data)
        theta3_data = pr.save_part_log(degrees(theta[2]), theta3_data)
        theta4_data = pr.save_part_log(degrees(theta[3]), theta4_data)
        theta_data = [theta1_data, theta2_data, theta3_data, theta4_data]

        dot_theta1_data = pr.save_part_log(dot_theta[0], dot_theta1_data)
        dot_theta2_data = pr.save_part_log(dot_theta[1], dot_theta2_data)
        dot_theta3_data = pr.save_part_log(dot_theta[2], dot_theta3_data)
        dot_theta4_data = pr.save_part_log(dot_theta[3], dot_theta4_data)
        dot_theta_data = [dot_theta1_data, dot_theta2_data,
                          dot_theta3_data, dot_theta4_data]

        ddot_theta1_data = pr.save_part_log(ddot_theta[0], ddot_theta1_data)
        ddot_theta2_data = pr.save_part_log(ddot_theta[1], ddot_theta2_data)
        ddot_theta3_data = pr.save_part_log(ddot_theta[2], ddot_theta3_data)
        ddot_theta4_data = pr.save_part_log(ddot_theta[3], ddot_theta4_data)
        ddot_theta_data = [ddot_theta1_data, ddot_theta2_data,
                           ddot_theta3_data, ddot_theta4_data]

        # Position data
        x_data = pr.save_part_log(position[0], x_data)
        xd_data = pr.save_part_log(xd, xd_data)
        y_data = pr.save_part_log(position[1], y_data)
        yd_data = pr.save_part_log(yd, yd_data)
        p_data = [x_data, y_data, xd_data, yd_data]
        xy_data = [y_data]

        sum_x_data = pr.save_part_log(sum_X[0], sum_x_data)
        sum_y_data = pr.save_part_log(sum_X[1], sum_y_data)

        # Binding Force Data
        lamx_data = pr.save_part_log(lam[0], lamx_data)
        lamy_data = pr.save_part_log(lam[1], lamy_data)
        lam_data = [lamx_data, lamy_data]

        fl.write(link_data)
        fm.write(motor_data)
        fp.write(position_data)

    # Print result
    title_name = ['Link angle', 'Motor angle',
                  'Time-Position', 'Binding force', 'Position',
                  'Non lineaar characteristics']

    label_name1 = ['Link1', 'Link2', 'Link3', 'Link4']
    label_name2 = ['Motor1', 'Motor2', 'Motor3', 'Motor4']
    label_name3 = ['X position', 'Y position', 'Xd position', 'Yd position']
    label_name4 = ['λx', 'λy']
    label_name5 = ["k1 = {}, k2 = {}". format(k1[0], k1[1])]
    label_name = [label_name1, label_name2, label_name3, label_name4,
                  label_name5]

    xlabel_name = ['Time[s]', 'X[m]', 'θ-q']
    ylabel_name = ['Angle[deg]', 'Position[m]', 'Force [N]', 'Y[m]', 'K']

    plt.figure(figsize=(9, 7))
    plt.subplot(321)
    pr.print_graph(title_name[0], time_log, q_data,
                   label_name[0], xlabel_name[0], ylabel_name[0],
                   num_plot_data=4)

    plt.subplot(322)
    pr.print_graph(title_name[1], time_log, theta_data,
                   label_name[1], xlabel_name[0], ylabel_name[0],
                   num_plot_data=4)

    plt.subplot(323)
    pr.print_graph(title_name[2], time_log, p_data,
                   label_name[2], xlabel_name[0], ylabel_name[1],
                   num_plot_data=4)

    plt.subplot(324)
    pr.print_graph(title_name[3], time_log, lam_data,
                   label_name[3], xlabel_name[0], ylabel_name[2],
                   num_plot_data=2)

    plt.subplot(325)
    pr.print_graph(title_name[4], p_data[0], xy_data, 'Position',
                   xlabel_name[1], ylabel_name[3], num_plot_data=1)
    plt.xlim(-0.8, 0.8)
    plt.ylim(-0.4, 0.8)

    plt.subplot(326)
    dif_data = [-3.14]
    K_part = []

    for i in range(628):
        d = -3.14 + i*0.01
        dif_data.append(d)
    for i in range(len(dif_data)):
        kpart1 = k1[0] + k1[1]*pow(dif_data[i], 2)
        kpart2 = kpart1*dif_data[i]
        K_part.append(kpart2)
        K_data = [K_part]

    pr.print_graph(title_name[5], dif_data, K_data, label_name[4],
                   xlabel_name[2], ylabel_name[4], num_plot_data=1)

    plt.show()

    fl.close()
    fm.close()
    fp.close()
