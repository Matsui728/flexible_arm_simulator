# -*- coding: utf-8 -*-
"""
Created on Mon Oct 30 15:49:51 2017

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
    m = [2, 2, 2, 2]  # 質量
    arm1_m = [m[0], m[1]]
    arm2_m = [m[2], m[3]]

    ll = [0.3, 0.3, 0.3, 0.3]  # リンク長さ
    link1 = [ll[0], ll[1]]
    link2 = [ll[2], ll[3]]
    L = [link1, link2]

    lg = [ll[0]/2, ll[1]/2, ll[2]/2, ll[3]/2]  # 重心位置
    arm1_lg = [lg[0], lg[1]]
    arm2_lg = [lg[2], lg[3]]

    D = 3.0  # リンク粘性
    g = 9.8  # 重力加速度

    # Gearratio
    N = 50

    # Prametas of motor
    Mm = 34.7*pow(10, -7)       # モータの慣性モーメント
    B = 0.0037255872020394314    # モータの粘性

    # 慣性モーメント
    Inertia = []
    Inertia = sl.link_inertia(m, ll, Inertia)

    arm1_Inertia = [Inertia[0], Inertia[1]]
    arm2_Inertia = [Inertia[2], Inertia[3]]

    # ゲイン調整
    control_gain1 = sl.imput_gain(0, 0.00, 0.000)
    control_gain2 = sl.imput_gain(0, 0.00, 0.000)
    control_gain3 = sl.imput_gain(0, 0.00, 0.000)
    control_gain4 = sl.imput_gain(0.0, 0.0, 0.0)
    gain1 = [control_gain1, control_gain2, control_gain3,
             control_gain4]

    control_gain1 = sl.imput_gain(0, 0.00, 0)
    control_gain2 = sl.imput_gain(0, 0.00, 0)
    control_gain3 = sl.imput_gain(0, 0.00, 0)
    control_gain4 = sl.imput_gain(0.0, 0.0, 0.0)
    gain2 = [control_gain1, control_gain2, control_gain3,
             control_gain4]

    # 初期姿勢
    x10 = 0.0
    y10 = 0.4

    rx0 = 0.1    # 第2アームのx軸分のずれ
    ry0 = 0.0    # 第2アームのy軸分のずれ

    r0 = [rx0, ry0]

    x20 = (x10 - rx0)
    y20 = (y10 - ry0)

    q, theta = ik.make_intial_angle_2dof(x10, y10, x20, y20,
                                         link1, link2)       # 初期角度

    theta = [N*theta[0], N*theta[1], N*theta[2], N*theta[3]]

    # Link data
    dot_q = [0.0, 0.0, 0.0, 0.0]
    ddot_q = [0.0, 0.0, 0.0, 0.0]
    sum_q = [0.0, 0.0, 0.0, 0.0]

    arm1_q = [q[0], q[1]]
    arm2_q = [q[2], q[3]]

    arm1_dotq = [dot_q[0], dot_q[1]]
    arm2_dotq = [dot_q[2], dot_q[3]]

    arm1_ddotq = [ddot_q[0], ddot_q[1]]
    arm2_ddotq = [ddot_q[2], ddot_q[3]]

    # Desired Parametas
    dot_qd = [0.0, 0.0, 0.0, 0.0]

    # Motor data
    dot_theta = [0.0, 0.0, 0.0, 0.0]
    ddot_theta = [0.0, 0.0, 0.0, 0.0]
    sum_theta = [0.0, 0.0, 0.0, 0.0]

    arm1_theta = [theta[0], theta[1]]
    arm2_theta = [theta[2], theta[3]]

    arm1_dottheta = [dot_theta[0], dot_theta[1]]
    arm2_dottheta = [dot_theta[2], dot_theta[3]]

    arm1_ddottheta = [ddot_theta[0], ddot_theta[1]]
    arm2_ddottheta = [ddot_theta[2], ddot_theta[3]]

    # Desired Parametas
    thetad = []    # 目標角度
    dot_thetad = [0.0, 0.0, 0.0, 0.0]

    # Input force
    f1_data, f2_data, f4_data = [], [], []
    Fconstant = 0
    force_gain = 0.0
    actf = []

    # Data list
    # nonliner_data
    nonliner_data = []
    (K_data1, K_data2, K_data3, K_data4) = [], [], [], []

    # linl data log
    (q1_data, q2_data, q3_data, q4_data) = [], [], [], []

    (qd1_data, qd2_data, qd3_data, qd4_data) = [], [], [], []

    (dot_q1_data, dot_q2_data, dot_q3_data,
     dot_q4_data) = [], [], [], []

    (ddot_q1_data, ddot_q2_data, ddot_q3_data,
     ddot_q4_data) = [], [], [], []

    (sum_q1_data, sum_q2_data, sum_q3_data,
     sum_q4_data) = [], [], [], []

    (dot_qd1_data, dot_qd2_data, dot_qd3_data,
     dot_qd4_data) = [], [], [], []

    (q_data, qd_data, dot_q_data,
     ddot_q_data, sum_q_data, dot_qd_data) = [], [], [], [], [], []

    # motor data log
    (theta1_data, theta2_data, theta3_data,
     theta4_data) = [], [], [], []

    (thetad1_data, thetad2_data, thetad3_data,
     thetad4_data) = [], [], [], []

    (dot_theta1_data, dot_theta2_data,
     dot_theta3_data, dot_theta4_data) = [], [], [], []

    (ddot_theta1_data, ddot_theta2_data,
     ddot_theta3_data, ddot_theta4_data) = [], [], [], []

    (theta21_data, theta22_data, theta23_data,
     theta24_data) = [], [], [], []

    (thetad21_data, thetad22_data, thetad23_data,
     thetad24_data) = [], [], [], []

    (dot_theta21_data, dot_theta22_data,
     dot_theta23_data, dot_theta24_data) = [], [], [], []

    (ddot_theta21_data, ddot_theta22_data,
     ddot_theta23_data, ddot_theta24_data) = [], [], [], []

    (sum_theta1_data, sum_theta2_data,
     sum_theta3_data, sum_theta4_data, sum_theta4_data) = [], [], [], [], []

    (theta_data, thetad_data, dot_theta_data,
     ddot_theta_data,
     sum_theta_data, dot_thetad_data) = [], [], [], [], [], []

    sum_polar = []

    circle_data = []

    k11 = 10
    k21 = 2000

    # Non linear character Parametas
    k1 = sl.non_linear_parameta(k11, k21)
    k2 = sl.non_linear_parameta(k11, k21)
    k3 = sl.non_linear_parameta(k11, k21)
    k4 = sl.non_linear_parameta(k11, k21)

    k = [k1, k2, k3, k4]

    # K Errored
    # +-5%誤差
    k11_error = uniform(9.5, 10.5)
    k21_error = uniform(1900, 2100)

    k1_error = sl.non_linear_parameta(9.5, 2100)
    k2_error = sl.non_linear_parameta(10.5, 1900)
    k3_error = sl.non_linear_parameta(10, 2000)
    k4_error = sl.non_linear_parameta(10, 2500)

    k_error = [k1_error, k2_error, k3_error,
               k4_error]

    # Time Parametas
    simulate_time = 10.0     # シミュレート時間
    sampling_time = 0.001  # サンプリングタイム
    time_log = []

    # Deseired Position
    xd1 = 0.4
    yd1 = 0.4

    xd2 = xd1 - rx0
    yd2 = yd1 - ry0

    thetaqq = radians(45)
    qd, thetaqd = ik.make_intial_angle_2dof(xd1, yd1, xd2, yd2,
                                            link1, link2)

    Xd = [xd1, yd1]
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

    eps = 0.8  # 判定半径
    eps1 = 0.05
    ST = int(sl.simulation_time(simulate_time, sampling_time))

    # file open
    fl = open('4dof_simulation_link_data.csv', 'w')
    fm = open('4dof_simulation_motor_data.csv', 'w')
    fp = open('4dof_simulation_position_data.csv', 'w')
    fc = open('Parameter_data.txt', 'w')

    fc.write('[Parameters]\n')


#   for i in range(len(gain)):
#       fc.write('Gain{} = [kp={}, kv={}, ki={}]\n'. format(i, gain[i][0],
#                                                           gain[i][1],
#                                                           gain[i][2]))

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
        # ヤコビとヤコビ転置
        J = sl.jacobi_matrix(ll, q)

        Jt = sl.transpose_matrix(J)

        # 手先位置導出
        X1 = ll[0] * cos(q[0]) + ll[1] * cos(q[0] + q[1])
        Y1 = ll[0] * sin(q[0]) + ll[1] * sin(q[0] + q[1])
        position1 = [X1, Y1]

        X2 = ll[2] * cos(q[2]) + ll[3] * cos(q[2] + q[3]) + r0[0]
        Y2 = ll[2] * sin(q[2]) + ll[3] * sin(q[2] + q[3]) + r0[1]
        position2 = [X2, Y2]

        # 偏差積分値の計算
        sum_q[0] = sl.sum_position_difference(sum_q[0], qd[0], q[0], sampling_time)
        sum_q[1] = sl.sum_position_difference(sum_q[1], qd[1], q[1], sampling_time)
        sum_q[2] = sl.sum_position_difference(sum_q[2], qd[2], q[2], sampling_time)
        sum_q[3] = sl.sum_position_difference(sum_q[3], qd[3], q[3], sampling_time)

        sum_q = [sum_q[0], sum_q[1], sum_q[2], sum_q[3]]

        Tau, actf, thetad = sl.PIDcontrol_eforce_base_3dof(gain1, gain2, theta, dot_theta,
                                                           Xd, position1, Jt, k, qd, sum_q, N,
                                                           force_gain, eps, Fconstant)

        # 偏差と非線形弾性特性値の計算
        e = sl.difference_part(theta, q, N)
        K = sl.non_linear_item(k, e)
        K_error = sl.non_linear_item(k_error, e)

        # 拘束力とダイナミクス右辺の計算
        dot_P, P, dot_Q, Q = sl.restraint_part(ll, q, dot_q, r0)

        Fx = sl.out_force(10, time)

        f, A, Tauff = sl.input_forces(ll, q, dot_q, H, D, K_error,
                                      Jt, P, Q, dot_P, dot_Q, N,
                                      0, 0, 10)

        # 関節角加速度の計算
        ddot_q = sl. angular_acceleration_3dof(invPhi, f, A)

        # 拘束項
        lam = sl.binding_force(invPhi, f, A)

        # モータ角加速度の計算
        ddot_theta = sl.motor_angular_acceleration(Mm, Tau, B,
                                                   dot_theta, K_error, N)

        C = sl.make_circle(eps1, time, xd1, yd1)

        # エクセル用log_data保存
#        link_data = pr.save_angle_excel_log(time, q, qd, dot_q, ddot_q)
#        motor_data = pr.save_angle_excel_log(time, theta, thetad,
#                                             dot_theta, ddot_theta)

        position_data = pr.save_position_log(time, position1[0], position1[1],
                                             xd1, yd1, lam[0], lam[1])

        # Save Circle data
        circlex_data = pr. save_part_log(C[0], circlex_data)
        circley_data = pr. save_part_log(C[1], circley_data)

        # Save time log
        time_log = pr.save_part_log(time, time_log)

        # Save K log
        K_data1 = pr.save_part_log(K_error[0], K_data1)
        K_data2 = pr.save_part_log(K_error[1], K_data2)
        K_data3 = pr.save_part_log(0, K_data3)
        K_data4 = pr.save_part_log(K_error[3], K_data4)
        non_liner_data = [K_data1, K_data2, K_data3, K_data4]

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
        dot_q_data = [dot_q1_data, dot_q2_data, dot_q3_data,
                      dot_q4_data]

        ddot_q1_data = pr.save_part_log(ddot_q[0], ddot_q1_data)
        ddot_q2_data = pr.save_part_log(ddot_q[1], ddot_q2_data)
        ddot_q3_data = pr.save_part_log(ddot_q[2], ddot_q3_data)
        ddot_q4_data = pr.save_part_log(ddot_q[3], ddot_q4_data)
        ddot_q_data = [ddot_q1_data, ddot_q2_data, ddot_q3_data,
                       ddot_q4_data]

        # Save Motor data(radian)
        thetad1_data = pr.save_part_log(degrees(thetad[0]), thetad1_data)
        thetad2_data = pr.save_part_log(degrees(thetad[1]), thetad2_data)
        thetad3_data = pr.save_part_log(degrees(thetad[2]), thetad3_data)
        thetad4_data = pr.save_part_log(degrees(thetad[3]), thetad4_data)
        thetad_data = [thetad1_data, thetad2_data, thetad3_data,
                       thetad4_data]

        theta1_data = pr.save_part_log(degrees(theta[0]), theta1_data)
        theta2_data = pr.save_part_log(degrees(theta[1]), theta2_data)
        theta3_data = pr.save_part_log(degrees(theta[2]), theta3_data)
        theta4_data = pr.save_part_log(degrees(theta[3]), theta4_data)

        theta_data = [theta1_data, theta2_data, theta3_data,
                      theta4_data]

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
        x_data = pr.save_part_log(position1[0], x_data)
        xd_data = pr.save_part_log(xd1, xd_data)
        y_data = pr.save_part_log(position1[1], y_data)
        yd_data = pr.save_part_log(yd1, yd_data)

        x2_data = pr.save_part_log(position2[0], x2_data)
        y2_data = pr.save_part_log(position2[1], y2_data)
        p_data = [x_data, y_data, x2_data, y2_data,
                  xd_data, yd_data]

        xyx_data = [x_data, xd_data, circlex_data]
        xyy_data = [y_data, yd_data, circley_data]


#        sum_x_data = pr.save_part_log(sum_X[0], sum_x_data)
#        sum_y_data = pr.save_part_log(sum_X[1], sum_y_data)

        # Binding Force Data
        lamx_data = pr.save_part_log(lam[0], lamx_data)
        lamy_data = pr.save_part_log(lam[1], lamy_data)
        lam_data = [lamx_data, lamy_data]

        # otherdata
        f1_data = pr.save_part_log(actf[0], f1_data)
        f2_data = pr.save_part_log(actf[1], f2_data)
        f4_data = pr.save_part_log(actf[2], f4_data)
        f_data = [f1_data, f2_data, f4_data]

        fl.write(link_data)
        fm.write(motor_data)
        fp.write(position_data)

    # Print result
    title_name = ['Link angle', 'Motor angle',
                  'Time-Position', 'Restraining force', 'Position',
                  'Non lineaar characteristics', 'Acting force',
                  'θd data', 'qd data', 'K(θ-q) data']

    label_name1 = ['Link1', 'Link2', 'Link3', 'Link4', 'Link5']
    label_name2 = ['Motor1', 'Motor2', 'Motor3', 'Motor4', 'Motor5']
    label_name3 = ['X1 position', 'Y1 position', 'X2 position', 'Y2 position',
                   'Xd position', 'Yd position']
    label_name4 = ['λx', 'λy']
    label_name5 = ["k1 = {}, k2 = {}". format(k1[0], k1[1])]
    label_name6 = ['f1', 'f2', 'f3', 'f5']
    label_name7 = ['Trajectory', 'Desired Position', 'Working circle']
    label_name8 = ['θd1','θd2','θd3','θd4','θd5']
    label_name9 = ['qd1','qd2','qd3','qd4','qd5']

    label_name = [label_name1, label_name2, label_name3, label_name4,
                  label_name5, label_name6, label_name7, label_name8,
                  label_name9]

    xlabel_name = ['Time[s]', 'X[m]', 'θ-q[rad]']
    ylabel_name = ['Angle[deg]', 'Position[m]', 'Force [N]', 'Y[m]', 'K[Nm]']

    plt.figure(figsize=(11, 12))
    plt.subplot(521)
    pr.print_graph(title_name[0], time_log, q_data,
                   label_name[0], xlabel_name[0], ylabel_name[0],
                   num_plot_data=4)

    plt.subplot(522)
    pr.print_graph(title_name[1], time_log, theta_data,
                   label_name[1], xlabel_name[0], ylabel_name[0],
                   num_plot_data=4)

    plt.subplot(523)
    pr.print_graph(title_name[2], time_log, p_data,
                   label_name[2], xlabel_name[0], ylabel_name[1],
                   num_plot_data=6)

    plt.subplot(524)
    pr.print_graph(title_name[3], time_log, lam_data,
                   label_name[3], xlabel_name[0], ylabel_name[2],
                   num_plot_data=2)

    plt.subplot(525)
    pr.print_graph_beta(title_name[4], xyx_data, xyy_data, label_name[6],
                        xlabel_name[1], ylabel_name[3], num_plot_data=2)
    plt.xlim(-0.6, 0.6)
    plt.ylim(0.0, 0.6)

    plt.subplot(526)

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


#    pr.print_graph(title_name[6], time_log, f_data,
#                   label_name[5], xlabel_name[0], ylabel_name[2],
#                   num_plot_data=4)

    plt.subplot(527)
    pr.print_graph(title_name[7], time_log, thetad_data,
                   label_name[7], xlabel_name[0], ylabel_name[0],
                   num_plot_data=4)

    plt.subplot(528)
    pr.print_graph(title_name[8], time_log, qd_data,
                   label_name[8], xlabel_name[0], ylabel_name[0],
                   num_plot_data=4)

    plt.subplot(529)
    pr.print_graph(title_name[9], time_log, non_liner_data,
                   label_name[0], xlabel_name[0], ylabel_name[4],
                   num_plot_data=4)

    plt.savefig('result.png')
    plt.show()

    fl.close()
    fm.close()
    fp.close()
    fc.close()

    pr.move_excel_data()
    print('Tasks are completed!')
    print("k1 = {}, k2 = {}". format(k11_error, k21_error))
