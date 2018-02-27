# -*- coding: utf-8 -*-
"""
Created on Fri Feb 16 15:07:13 2018

@author: kawalab
"""

import numpy as np
import matplotlib.pyplot as plt
import configparser
import print_result as pr
import simlib as sl
import inverse_kinematics as ik
from tqdm import tqdm
from math import degrees, radians, sin, cos, atan2, sqrt
from random import uniform



# データログ保存パス
cp = configparser.ConfigParser()
cp.read('config')
root_dir = cp.get('dataset_dir', 'dir_path')


if __name__ == '__main__':
    # Parameters
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

    D = 3.0  # リンク粘性
    g = 9.8  # 重力加速度


    # Gearratio
    N = 50

    # Prametas of motor
    Mm = 34.7*pow(10, -7)       # モータの慣性モーメント
    B = 0.0037255872020394314 # モータの粘性

    # 慣性モーメント
    Inertia = []
    Inertia = sl.link_inertia(m, ll, Inertia)

    arm1_Inertia = [Inertia[0], Inertia[1], Inertia[2]]
    arm2_Inertia = [Inertia[3], Inertia[4]]

    # ゲイン調整
    control_gain1 = sl.imput_gain(0, 0.00, 0.000)
    control_gain2 = sl.imput_gain(0, 0.00, 0.000)
    control_gain3 = sl.imput_gain(0, 0.00, 0.000)
    control_gain4 = sl.imput_gain(0.0, 0.0, 0.0)
    control_gain5 = sl.imput_gain(0, 0.00, 0.000)
    gain1 = [control_gain1, control_gain2, control_gain3,
            control_gain4, control_gain5]

    control_gain1 = sl.imput_gain(0, 0.00, 0)
    control_gain2 = sl.imput_gain(0, 0.00, 0)
    control_gain3 = sl.imput_gain(0, 0.00, 0)
    control_gain4 = sl.imput_gain(0.0, 0.0, 0.0)
    control_gain5 = sl.imput_gain(0, 0.00, 0)
    gain2 = [control_gain1, control_gain2, control_gain3,
             control_gain4, control_gain5]

    # 初期姿勢
    x0 = 0.0
    y0 = 0.4
    Theta = atan2(y0,x0) -radians(90)
    xd = 0.0
    yd = 0.4
    thetaqq = atan2(yd,xd) -radians(90)

    q, theta = ik.make_intial_angle(x0, y0, link1, link2, Theta)  # 初期角度

    theta = [N*theta[0], N*theta[1], N*theta[2],
             N*theta[3], N*theta[4]]
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

    # Motor data
    dot_theta = [0.0, 0.0, 0.0, 0.0, 0.0]
    ddot_theta = [0.0, 0.0, 0.0, 0.0, 0.0]
    sum_theta = [0.0, 0.0, 0.0, 0.0, 0.0]

    arm1_theta = [theta[0], theta[1], theta[2]]
    arm2_theta = [theta[3], theta[4]]

    arm1_dottheta = [dot_theta[0], dot_theta[1], dot_theta[2]]
    arm2_dottheta = [dot_theta[3], dot_theta[4]]

    arm1_ddottheta = [ddot_theta[0], ddot_theta[1], ddot_theta[2]]
    arm2_ddottheta = [ddot_theta[3], ddot_theta[4]]

    # Desired Parametas
    thetad = []    # 目標角度
    dot_thetad = [0.0, 0.0, 0.0, 0.0, 0.0]

    # Input force
    f1_data, f2_data, f3_data, f5_data = [], [], [], []
    Fconstant = 100
    force_gain = 0.0
    actf = []

    # Data list
    # nonliner_data
    nonliner_data = []
    (K_data1, K_data2, K_data3, K_data4, K_data5) = [], [], [], [], []

    # linl data log
    (q1_data, q2_data, q3_data, q4_data, q5_data) = [], [], [], [], []

    (qd1_data, qd2_data, qd3_data, qd4_data, qd5_data) = [], [], [], [], []

    (dot_q1_data, dot_q2_data, dot_q3_data,
     dot_q4_data, dot_q5_data) = [], [], [], [], []

    (ddot_q1_data, ddot_q2_data, ddot_q3_data,
     ddot_q4_data, ddot_q5_data) = [], [], [], [], []

    (sum_q1_data, sum_q2_data, sum_q3_data,
     sum_q4_data, sum_q5_data) = [], [], [], [], []

    (dot_qd1_data, dot_qd2_data, dot_qd3_data,
     dot_qd4_data, dot_qd5_data) = [], [], [], [], []

    (q_data, qd_data, dot_q_data,
     ddot_q_data, sum_q_data, dot_qd_data) = [], [], [], [], [], []

    # motor data log
    (theta1_data, theta2_data, theta3_data,
     theta4_data, theta5_data) = [], [], [], [], []

    (thetad1_data, thetad2_data, thetad3_data,
     thetad4_data, thetad5_data) = [], [], [], [], []

    (dot_theta1_data, dot_theta2_data,
     dot_theta3_data, dot_theta4_data, dot_theta5_data) = [], [], [], [], []

    (ddot_theta1_data, ddot_theta2_data,
     ddot_theta3_data, ddot_theta4_data, ddot_theta5_data) = [], [], [], [], []

    (theta21_data, theta22_data, theta23_data,
     theta24_data, theta25_data) = [], [], [], [], []

    (thetad21_data, thetad22_data, thetad23_data,
     thetad24_data, thetad25_data) = [], [], [], [], []

    (dot_theta21_data, dot_theta22_data,
     dot_theta23_data, dot_theta24_data, dot_theta25_data) = [], [], [], [], []

    (ddot_theta21_data, ddot_theta22_data,
     ddot_theta23_data, ddot_theta24_data, ddot_theta25_data) = [], [], [], [], []

    (sum_theta1_data, sum_theta2_data,
     sum_theta3_data, sum_theta4_data, sum_theta4_data) = [], [], [], [], []

    (theta_data, thetad_data, dot_theta_data,
     ddot_theta_data,
     sum_theta_data, dot_thetad_data) = [], [], [], [], [], []


    deviation_data = []         # 偏差データ
    deviation_X_data = []       # 偏差ｘデータ
    deviation_Y_data = []       # 偏差yデータ

    DX, DY, DR = [],[],[]


    # 偏差外力データ
    force_angle = [radians(0), radians(45),
                   radians(90), radians(135),
                   radians(180), radians(225),
                   radians(270), radians(315)]


    sum_polar = []

    circle_data = []

    k11 = 10
    k21 = 2000

    # Non linear character Parametas
    k1 = sl.non_linear_parameta(k11, k21)
    k2 = sl.non_linear_parameta(k11, k21)
    k3 = sl.non_linear_parameta(k11, k21)
    k4 = sl.non_linear_parameta(k11, k21)
    k5 = sl.non_linear_parameta(k11, k21)

    k = [k1, k2, k3, k4, k5]

    # K Errored
    # +-5%誤差
    k11_error = uniform(9.5, 10.5)
    k21_error = uniform(1900, 2100)

    k1_error = sl.non_linear_parameta(9.5, 2100)
    k2_error = sl.non_linear_parameta(10.5, 1900)
    k3_error = sl.non_linear_parameta(10.5,2000)
    k4_error = sl.non_linear_parameta(10.5,2100)
    k5_error = sl.non_linear_parameta(9.5, 1900)

    k_error = [k1_error, k2_error, k3_error,
               k4_error, k5_error]


    # Time Parametas
    simulate_time = 4.0     # シミュレート時間
    sampling_time = 0.001  # サンプリングタイム
    time_log = []

    # Deseired Position
    qd, thetaqd = ik.make_intial_angle(xd, yd, link1, link2, thetaqq)
    Rd = sqrt(pow(xd, 2) + pow(yd, 2))
    phid = radians(atan2(yd, xd))
    Xd = [xd, yd]
    x_data = []
    xd_data = []
    yd_data = []
    y_data = []
    x2_data = []
    y2_data = []
    c_data = []
    circlex_data = []
    circley_data = []
    sum_x = 0.0
    sum_y = 0.0
    sum_r = 0.0
    sum_phi = 0.0
    sum_x_data = []
    sum_y_data = []

    lamx_data = []
    lamy_data = []

    eps = 0.3  # 判定半径
    eps1 = 0.05
    ST = int(sl.simulation_time(simulate_time, sampling_time))

    # file open
    fl = open('4dof_simulation_link_data.csv', 'w')
    fm = open('4dof_simulation_motor_data.csv', 'w')
    fp = open('4dof_simulation_position_data.csv', 'w')
    fc = open('Parameter_data.txt', 'w')

    fc.write('[Parameters]\n')
    fc.write('Xd = {}, Yd ={}\n'. format(Xd[0], Xd[1])
             + 'k1 = [{},{},{},{},{}],'. format(k[0][0], k[1][0],
                                                k[2][0], k[3][0], k[4][0])
             + 'k2 = [{},{},{},{},{}]\n'. format(k[0][1], k[1][1],
                                                 k[2][1], k[3][1], k[4][1]))

 #   for i in range(len(gain)):
 #       fc.write('Gain{} = [kp={}, kv={}, ki={}]\n'. format(i, gain[i][0],
 #                                                           gain[i][1],
 #                                                            gain[i][2]))

    fc.write('simulate time = {}[s]'. format(simulate_time))
    fc.write('sampling time = {}[s]\n'. format(sampling_time))
    fc.write('fc = {}\n'. format(Fconstant))
    fl.write('Time[s], q1, q2, q3, q4,q5, qd1, qd2, qd3, qd4, qd5'
             + 'dot_q1, dot_q2, dot_q3, dot_q4, dot_q5'
             + 'ddot_q1, ddot_q2, ddot_q3, ddot_q4, ddot_q5\n')

    fm.write('Time[s], theta1, theta2, theta3, theta4, theta5'
             + 'thetad1, thetad2, thetad3, thetad4, thetad5'
             + 'dot_theta1, dot_theta2, dot_theta3, dot_theta4, dot_theta5'
             + 'ddot_theta1, ddot_theta2, ddot_theta3, ddot_theta4, ddot_theta5\n')

    fp.write('Time[s], X, Y, Xd, Yd, lambdax, lambday\n')

    for j in tqdm(range(len(force_angle))):
        fangle = force_angle[j]

        fx = 10*cos(fangle)
        fy = 10*sin(fangle)


        for i in tqdm(range(ST)):

            time = i*sampling_time  # 時間の設定

            # オイラー法の定義
            q, dot_q, ddot_q = sl.EulerMethod(q, dot_q, ddot_q, sampling_time)

            theta, dot_theta, ddot_theta = sl.EulerMethod(theta, dot_theta,
                                                          ddot_theta,
                                                          sampling_time)

            # 慣性行列の定義
            mm1 = sl.moment_matrix_serial3dof(arm1_m, link1, arm1_lg,
                                              arm1_Inertia, arm1_q)

            mm2 = sl.moment_matrix_serial2dof(arm2_m, link2, arm2_lg,
                                              arm2_Inertia, arm2_q)

            M = [mm1, mm2]

            # コリオリ項の定義
            H1 = sl.coriolis_item_serial3dof(arm1_m, link1,
                                             arm1_lg, arm1_q, arm1_dotq)

            H2 = sl.coriolis_item_serial2dof(arm2_m, link2,
                                             arm2_lg, arm2_q, arm2_dotq)

            H = [H1, H2]

            # 重力項の定義
            g1, g2, g3, g4, g5 = 0.0, 0.0, 0.0, 0.0, 0.0
            G = [g1, g2, g3, g4, g5]

            # 二回微分値の導出
            E = sl.define_E(ll, q)

            # 逆行列の掃き出し
            Phi = sl.phi_matrix_4dof(mm1, mm2, E)
            invPhi = sl.inverse_matrix(Phi)

            # ヤコビとヤコビ転置
            J1 = sl.jacobi_serial3dof(link1, arm1_q)
            J2 = sl.jacobi_serial2dof(link2, arm2_q)
            J = sl.jacobi(J1, J2)
            Jt = sl.transpose_matrix(J)

            # 極座標系ヤコビ行列
            J1_polar = sl.jacobi_polar_coordinates_3dof(link1, arm1_q)
            J2_polar = sl.jacobi_polar_coordinates_2dof(link2, arm2_q)
            J_polar = sl.jacobi(J1_polar, J2_polar)
            Jt_polar = sl.transpose_matrix(J_polar)



            # 手先位置導出
            X = ll[0] * cos(q[0]) + ll[1] * cos(q[0] + q[1]) + ll[2] * cos(q[0] + q[1] + q[2])
            Y = ll[0] * sin(q[0]) + ll[1] * sin(q[0] + q[1]) + ll[2] * sin(q[0] + q[1] + q[2])
            position = [X, Y]

            X2 = ll[3]*cos(q[3]) + ll[4]*cos(q[3]+q[4])
            Y2 = ll[3]*sin(q[3]) + ll[4]*sin(q[3]+q[4])
            position2 = [X2, Y2]

            R = sqrt(pow(X, 2) + pow(Y, 2))
            phi = radians(atan2(Y, X))
            phi1 = radians(q[0] + q[1] + q[2])
            phi2 = radians(q[3] + q[4])

            # 偏差積分値の計算
            sum_r = sl.sum_position_difference(sum_r, Rd, R, sampling_time)
            sum_phi = sl.sum_position_difference(sum_phi, phid, phi, sampling_time)
            sum_polar = [sum_r, sum_phi]
            sum_q[0] = sl.sum_position_difference(sum_q[0], qd[0], q[0], sampling_time)
            sum_q[1] = sl.sum_position_difference(sum_q[1], qd[1], q[1], sampling_time)
            sum_q[2] = sl.sum_position_difference(sum_q[2], qd[2], q[2], sampling_time)
            sum_q[3] = sl.sum_position_difference(sum_q[3], qd[3], q[3], sampling_time)
            sum_q[4] = sl.sum_position_difference(sum_q[4], qd[4], q[4], sampling_time)

            sum_q = [sum_q[0], sum_q[1], sum_q[2], sum_q[3], sum_q[4]]
    #        Tau, actf, thetad = sl.PIDcontrol_polar(gain1, gain2, theta, dot_theta, Xd, position, Jt, Jt_polar,
    #                                                k, qd, Rd, R, phid, phi, phi1, phi2,
    #                                                force_gain, eps, Fconstant)

            Tau, actf, thetad = sl.PIDcontrol_eforce_base(gain1, gain2, theta, dot_theta,
                                                          Xd, position, Jt, k, qd, sum_q, N,
                                                          force_gain, eps, Fconstant)

            # 偏差と非線形弾性特性値の計算
            e = sl.difference_part(theta, q, N)
            K = sl.non_linear_item(k, e)
            K_error = sl.non_linear_item(k_error, e)

            # 拘束力とダイナミクス右辺の計算
            dot_P, P, dot_Q, Q = sl.restraint_item(ll, q, dot_q)

            Fx = sl.out_force(10, time)

            f, A, Tauff = sl.input_forces_4dof(ll, q, dot_q, H, D, K,
                                               Jt, P, Q, dot_P, dot_Q, N,
                                               fx, fy, 10)

            # 関節角加速度の計算
            ddot_q = sl.angular_acceleration_4dof(invPhi, f, A)

            # 拘束項
            lam = sl.binding_force_4dof(invPhi, f, A)

            # モータ角加速度の計算
            ddot_theta = sl.motor_angular_acceleration(Mm, Tau, B,
                                                       dot_theta, K, N)

            C = sl.make_circle(eps, time, xd, yd)

            # エクセル用log_data保存
            link_data = pr.save_angle_excel_log(time, q, qd, dot_q, ddot_q)
            motor_data = pr.save_angle_excel_log(time, theta, thetad,
                                                 dot_theta, ddot_theta)

            position_data = pr.save_position_log(time, position[0], position[1],
                                                 xd, yd, lam[0], lam[1])

            # Save Circle data
            circlex_data = pr. save_part_log(C[0], circlex_data)
            circley_data = pr. save_part_log(C[1], circley_data)

            # Save time log
            time_log = pr.save_part_log(time, time_log)

            # Save K log
            K_data1 = pr.save_part_log(K_error[0], K_data1)
            K_data2 = pr.save_part_log(K_error[1], K_data2)
            K_data3 = pr.save_part_log(K_error[2], K_data3)
            K_data4 = pr.save_part_log(0, K_data4)
            K_data5 = pr.save_part_log(K_error[4], K_data5)
            non_liner_data = [K_data1, K_data2, K_data3, K_data4, K_data5]

            # Save link_data(radian)
            qd1_data = pr.save_part_log(degrees(qd[0]), qd1_data)
            qd2_data = pr.save_part_log(degrees(qd[1]), qd2_data)
            qd3_data = pr.save_part_log(degrees(qd[2]), qd3_data)
            qd4_data = pr.save_part_log(degrees(qd[3]), qd4_data)
            qd5_data = pr.save_part_log(degrees(qd[4]), qd5_data)
            qd_data = [qd1_data, qd2_data, qd3_data, qd4_data, qd5_data]

            q1_data = pr.save_part_log(degrees(q[0]), q1_data)
            q2_data = pr.save_part_log(degrees(q[1]), q2_data)
            q3_data = pr.save_part_log(degrees(q[2]), q3_data)
            q4_data = pr.save_part_log(degrees(q[3]), q4_data)
            q5_data = pr.save_part_log(degrees(q[4]), q5_data)
            q_data = [q1_data, q2_data, q3_data, q4_data, q5_data,
                      qd1_data, qd2_data, qd3_data, qd4_data, qd5_data]

            dot_q1_data = pr.save_part_log(dot_q[0], dot_q1_data)
            dot_q2_data = pr.save_part_log(dot_q[1], dot_q2_data)
            dot_q3_data = pr.save_part_log(dot_q[2], dot_q3_data)
            dot_q4_data = pr.save_part_log(dot_q[3], dot_q4_data)
            dot_q5_data = pr.save_part_log(dot_q[4], dot_q5_data)
            dot_q_data = [dot_q1_data, dot_q2_data, dot_q3_data,
                          dot_q4_data, dot_q5_data]

            ddot_q1_data = pr.save_part_log(ddot_q[0], ddot_q1_data)
            ddot_q2_data = pr.save_part_log(ddot_q[1], ddot_q2_data)
            ddot_q3_data = pr.save_part_log(ddot_q[2], ddot_q3_data)
            ddot_q4_data = pr.save_part_log(ddot_q[3], ddot_q4_data)
            ddot_q5_data = pr.save_part_log(ddot_q[4], ddot_q5_data)
            ddot_q_data = [ddot_q1_data, ddot_q2_data, ddot_q3_data,
                           ddot_q4_data, ddot_q5_data]

            # Save Motor data(radian)
            thetad1_data = pr.save_part_log(degrees(thetad[0]), thetad1_data)
            thetad2_data = pr.save_part_log(degrees(thetad[1]), thetad2_data)
            thetad3_data = pr.save_part_log(degrees(thetad[2]), thetad3_data)
            thetad4_data = pr.save_part_log(degrees(thetad[3]), thetad4_data)
            thetad5_data = pr.save_part_log(degrees(thetad[4]), thetad5_data)
            thetad_data = [thetad1_data, thetad2_data, thetad3_data,
                           thetad4_data, thetad5_data]

            theta1_data = pr.save_part_log(degrees(theta[0]), theta1_data)
            theta2_data = pr.save_part_log(degrees(theta[1]), theta2_data)
            theta3_data = pr.save_part_log(degrees(theta[2]), theta3_data)
            theta4_data = pr.save_part_log(degrees(theta[3]), theta4_data)
            theta5_data = pr.save_part_log(degrees(theta[4]), theta5_data)
            theta_data = [theta1_data, theta2_data, theta3_data,
                          theta4_data, theta5_data,
                          thetad1_data, thetad2_data, thetad3_data,
                          thetad4_data, thetad5_data]

            dot_theta1_data = pr.save_part_log(dot_theta[0], dot_theta1_data)
            dot_theta2_data = pr.save_part_log(dot_theta[1], dot_theta2_data)
            dot_theta3_data = pr.save_part_log(dot_theta[2], dot_theta3_data)
            dot_theta4_data = pr.save_part_log(dot_theta[3], dot_theta4_data)
            dot_theta5_data = pr.save_part_log(dot_theta[4], dot_theta5_data)
            dot_theta_data = [dot_theta1_data, dot_theta2_data,
                              dot_theta3_data, dot_theta4_data,
                              dot_theta5_data]

            ddot_theta1_data = pr.save_part_log(ddot_theta[0], ddot_theta1_data)
            ddot_theta2_data = pr.save_part_log(ddot_theta[1], ddot_theta2_data)
            ddot_theta3_data = pr.save_part_log(ddot_theta[2], ddot_theta3_data)
            ddot_theta4_data = pr.save_part_log(ddot_theta[3], ddot_theta4_data)
            ddot_theta5_data = pr.save_part_log(ddot_theta[4], ddot_theta5_data)
            ddot_theta_data = [ddot_theta1_data, ddot_theta2_data,
                               ddot_theta3_data, ddot_theta4_data,
                               ddot_theta5_data]

            # Position data
            x_data = pr.save_part_log(position[0], x_data)
            xd_data = pr.save_part_log(xd, xd_data)
            y_data = pr.save_part_log(position[1], y_data)
            yd_data = pr.save_part_log(yd, yd_data)

            x2_data = pr.save_part_log(position2[0], x2_data)
            y2_data = pr.save_part_log(position2[1], y2_data)
            p_data = [x_data, y_data, xd_data, yd_data]

            xyx_data = [x_data, xd_data, circlex_data]
            xyy_data = [y_data, yd_data, circley_data]


    #        sum_x_data = pr.save_part_log(sum_X[0], sum_x_data)
    #        sum_y_data = pr.save_part_log(sum_X[1], sum_y_data)

            # Binding Force Data
            lamx_data = pr.save_part_log(lam[0], lamx_data)
            lamy_data = pr.save_part_log(lam[1], lamy_data)
            lam_data = [lamx_data, lamy_data]

            if i == ST-1:
                rfx = position[0]
                rfy = position[1]
                rxy = sqrt(pow(x0-position[0], 2)+pow(y0-position[1], 2))

                deviation_X_data = rfx
                deviation_Y_data = rfy
                deviation_R_data = rxy

            # otherdata
            f1_data = pr.save_part_log(actf[0], f1_data)
            f2_data = pr.save_part_log(actf[1], f2_data)
            f3_data = pr.save_part_log(actf[2], f3_data)
            f5_data = pr.save_part_log(actf[3], f5_data)
            f_data = [f1_data, f2_data, f3_data, f5_data]

            fl.write(link_data)
            fm.write(motor_data)
            fp.write(position_data)

        DX.append(deviation_X_data)
        DY.append(deviation_Y_data)
        DR.append(deviation_R_data)

    # Print result
    title_name = ['Link angle', 'Motor angle',
                  'Time-Position', 'Restraining force', 'Position',
                  'Position Error']

    label_name1 = ['Link1', 'Link2', 'Link3', 'Link4', 'Link5']
    label_name2 = ['Motor1', 'Motor2', 'Motor3', 'Motor4', 'Motor5']
    label_name3 = ['X position', 'Y position',
                   'Xd position', 'Yd position']
    label_name4 = ['λx', 'λy']
    label_name5 = ["k1 = {}, k2 = {}". format(k1[0], k1[1])]
    label_name6 = ['f1', 'f2', 'f3', 'f5']
    label_name7 = ['Trajectory', 'Desired Position', 'Working circle']
    label_name8 = ['θd1','θd2','θd3','θd4','θd5']
    label_name9 = ['qd1','qd2','qd3','qd4','qd5']
    label_name10 = ['q1','q2','q3','q4','q5','qd1','qd2','qd3','qd4','qd5']
    label_name11 = ['θ1','θ2','θ3','θ4','θ5', 'θd1','θd2','θd3','θd4','θd5']
    label_name12 = ['0[deg]','45[deg]','90[deg]','135[deg]','180[deg]', '225[deg]','270[deg]','315[deg]']
    label_name = [label_name1, label_name2, label_name3, label_name4,
                  label_name5, label_name6, label_name7, label_name8,
                  label_name9, label_name10, label_name11, label_name12]

    xlabel_name = ['Time[s]', 'X[m]', 'θ-q[rad]']
    ylabel_name = ['Angle[deg]', 'Position[m]', 'Force [N]', 'Y[m]', 'K(θ-q)[Nm]']

    plt.figure(figsize=(5, 5))

    pr.print_graph_gamma(title_name[5], DX, DY, label_name[11],
                   xlabel_name[1], ylabel_name[3], num_plot_data=8)
    plt.xlim(-0.6, 0.6)
    plt.ylim(-0.6, 0.6)
    plt.savefig('result1.png')
    plt.show()


    # 軸比計算
    var_max = max(DR)
    var_min = min(DR)
    var = var_min/var_max

    fl.close()
    fm.close()
    fp.close()
    fc.close()

    print('Tasks are completed!')
    print("X = {}". format(x0))
    print("Y = {}". format(y0))
    print("Axial ratio = {}". format(var))

