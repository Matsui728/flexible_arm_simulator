# -*- coding: utf-8 -*-
"""
Created on Thu Jan 25 13:55:51 2018

@author: kawalab
"""

import numpy as np
import matplotlib.pyplot as plt
import print_result as pr
import simlib as sl
from math import degrees, radians, sin, cos, atan2, sqrt, pow
from tqdm import tqdm


if __name__ == '__main__':
    # パラメータ
    ll = [0.3]  # リンク長
    lg = [ll[0] / 2]   # 重心位置
    m = [2]
    Mm = 34.7*pow(10, -7)       # モータの慣性モーメント

    D = 3   # リンク粘性
    B = 0.0011781341180018513 # モータ粘性
    g = 9.8  # 重力加速度
    Kf = 0.00

    Inertia = []
    Inertia = sl.link_inertia(m, ll, Inertia)

    # ロボット初期値
    theta = [radians(0.0)]
    thetad1 = radians(0.0)
    dot_theta = [0.0]
    ddot_theta = [0.0]

    q = [radians(30.0)]
    dot_q = [0]
    ddot_q = [0]

    k1 = 10
    k2 = 2000
    k11 = sl.non_linear_parameta(k1, k2)
    k = [k11]

    kp = 100
    kv = 00

    tauf = 0  # 外乱トルク

    Gear_ratio = 100

    thetad2 = Gear_ratio * thetad1

    A = m[0] * pow(lg[0], 2) + Inertia[0]

    # リスト作成
    (q_data, dot_q_data, ddot_q_data) = [], [], []
    K_data = []
    L_data = []
    tauf_data = []

    # Time Parametas
    simulate_time = 3.0     # シミュレート時間
    sampling_time = 0.001  # サンプリングタイム
    time_log = []

    ST = int(sl.simulation_time(simulate_time, sampling_time))

    for j in range(200):
        tauf = 0.1 * j

        for i in tqdm(range(ST)):
            time = i*sampling_time  # 時間の設定

            q, dot_q, ddot_q = sl.EulerMethod(q, dot_q, ddot_q, sampling_time)
    #        theta, dot_theta, ddot_theta = sl.EulerMethod(theta, dot_theta,
    #                                                      ddot_theta,
    #                                                      sampling_time)

    #        tau = kp * (thetad2 - theta[0]) - kv * dot_theta[0]

            # 偏差と非線形弾性特性値の計算
            e = sl.difference_part(theta, q, Gear_ratio)
            K = sl.non_linear_item(k, e)

            # 各角加速度計算

            ddot_q[0] = (K[0] + tauf - D * dot_q[0]) / A
    #        ddot_theta[0] = (Gear_ratio * tau - K[0] - pow(Gear_ratio, 2) * B * dot_theta[0]) / pow(Gear_ratio, 2) / Mm

            # Save time log
            time_log = pr.save_part_log(time, time_log)

            q_data = pr.save_part_log(degrees(q[0]), q_data)
            qdata = [q_data]
            dot_q_data = pr.save_part_log(degrees(dot_q[0]), dot_q_data)
            ddot_q_data = pr.save_part_log(degrees(ddot_q[0]), ddot_q_data)

        x = ll[0] * cos(q[0])
        y = ll[0] * sin(q[0])

        r = sqrt(pow(ll[0]-x, 2) + pow(0-y, 2))

        L = 2 * ll[0] * 3.14 * degrees(q[0])/360

        tauf_data.append(tauf)
        L_data.append(L)

    l_data = [L_data]


    title_name = ['Force-Deviation']
    label_name = ['q']
    xlabel_name = ['Time[s]', 'X[m]', 'θ-q[rad]', 'External force[N]']
    ylabel_name = ['Angle[deg]', 'Position[m]',
                   'Y[m]', 'K(θ-q)[Nm]', 'Deviation[m]']
    plt.figure(figsize=(6, 3))
    pr.print_graph(title_name[0], tauf_data, l_data,
                   label_name[0], xlabel_name[3], ylabel_name[4],
                   num_plot_data=1)


    print(r)
    print(L)
