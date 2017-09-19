# -*- coding: utf-8 -*-
"""
Created on Sat Sep  9 16:32:10 2017

@author: kawalab
"""

import numpy as np
import matplotlib.pyplot as plt
from math import degrees, radians, sin, cos
import print_result as pr
import simlib as sl


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
    control_gain1 = sl.imput_gain(1.0, 0.004, 0.0)
    control_gain2 = sl.imput_gain(1.0, 0.004, 0.0)
    control_gain3 = sl.imput_gain(0.0, 0.0, 0.0)
    control_gain4 = sl.imput_gain(1.0, 0.004, 0.0)
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

    # Non linear character Parametas
    k1 = sl.non_linear_parameta(0.003, 0.003)
    k2 = sl.non_linear_parameta(0.003, 0.003)
    k3 = sl.non_linear_parameta(0.000, 0.000)
    k4 = sl.non_linear_parameta(0.003, 0.003)

    k = [k1, k2, k3, k4]

    # Time Parametas
    simulate_time = 10      # シミュレート時間
    sampling_time = 0.001  # サンプリングタイム

    # Deseired Position
    xd = 0.5
    yd = 0.2
    Xd = [xd, yd]
    x_data = []
    y_data = []
    sum_x = []
    sum_y = []

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

        # 手先位置導出
        X = ll[0] * cos(q[0]) + ll[1] * cos(q[0] + q[1])
        Y = ll[0] * sin(q[0]) + ll[1] * sin(q[0] + q[1])

        potision = [X, Y]

        # 偏差積分値の計算
        # sum_x = sl.sum_position_difference(sum_x, xd, X, sampling_time)
        # sum_y = sl.sum_position_difference(sum_y, yd, Y, sampling_time)
        sum_x = 0.0
        sum_y = 0.0

        sum_X = [sum_x, sum_y]

        # モータ入力
        Tau = sl.PID_potiton_control_3dof(gain, Xd, potision,
                                          Jt, dot_theta, sum_X)

        # 偏差と非線形弾性特性値の計算
        e = sl.difference_part(theta, q)

        K = sl.non_linear_item(k[:][0], k[:][1], e)

        # 拘束力とダイナミクス右辺の計算
        dot_P, P, dot_Q, Q = sl.restraint_part(ll, q, dot_q)

        f, A = sl.input_forces(ll, q, dot_q, H, D, K,
                               Jt, P, Q, dot_P, dot_Q, Fx=0, Fy=0, s=30)

        # 関節角加速度の計算
        ddot_q = sl.angular_acceleration_3dof(invPhi, f, A)

        # 拘束項
        lam = sl.binding_force(invPhi, f, A)

        # モータ角加速度の計算
        ddot_theta = sl.motor_angular_acceleration(Mm, Tau[i-1], B,
                                                   dot_theta[i-1], K)

        # エクセル用log_data保存
        link_data = pr.save_angle_excel_log(q, qd, dot_q, ddot_q)
        motor_data = pr.save_angle_excel_log(theta, thetad,
                                             dot_theta, ddot_theta)

        position_data = pr.save_position_log(time, potision[0], potision[1],
                                             xd, yd, lam[0], lam[1])

        # Save time log
        time_log = pr.save_part_log(time, time_log)

        # Save link_data(radian)
        qd_data = pr.make_data_log_list(qd)
        q_data = pr.make_data_log_list(q)
        dot_q_data = pr.make_data_log_list(dot_q)
        ddot_q_data = pr.make_data_log_list(ddot_q)

        # Save Motor data(radian)
        thetad_data = pr.make_data_log_list(thetad)
        theta_data = pr.make_data_log_list(theta)
        dot_theta_data = pr.make_data_log_list(dot_theta)
        ddot_theta_data = pr.make_data_log_list(ddot_theta)

        # Position data
        x_data = pr.save_part_log(potision[0], x_data)
        y_data = pr.save_part_log(potision[1], y_data)

        fl.write(link_data)
        fm.write(motor_data)
        fp.write(position_data)

    # Print result


    fl.close()
    fm.close()
