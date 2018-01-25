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
    m = [0.3]

    D = 0.4   # リンク粘性
    g = 9.8  # 重力加速度

    Inertia = []
    Inertia = sl.link_inertia(m, ll, Inertia)

    # ロボット初期値
    theta = [0.0]
    q = [radians(0.0)]
    dot_q = [0]
    ddot_q = [0]

    k1 = 40
    k2 = 800

    k11 = sl.non_linear_parameta(k1, k2)

    k = [k11]
    tauf = 10    # 外乱トルク

    A = m[0] * pow(lg[0], 2) + Inertia[0]

    # リスト作成
    (q_data, dot_q_data, ddot_q_data) = [], [], []
    K_data = []

    # Time Parametas
    simulate_time = 2.0     # シミュレート時間
    sampling_time = 0.001  # サンプリングタイム
    time_log = []
    ST = int(sl.simulation_time(simulate_time, sampling_time))

    for i in tqdm(range(ST)):
        time = i*sampling_time  # 時間の設定

        q, dot_q, ddot_q = sl.EulerMethod(q, dot_q, ddot_q, sampling_time)

        # 偏差と非線形弾性特性値の計算
        e = sl.difference_part(theta, q)
        K = sl.non_linear_item(k, e)

        # 各加速度計算
        ddot_q[0] = (K[0] + tauf - D * dot_q[0]) / A

        # Save time log
        time_log = pr.save_part_log(time, time_log)

        q_data = pr.save_part_log(degrees(q[0]), q_data)
        qdata = [q_data]
        dot_q_data = pr.save_part_log(degrees(dot_q[0]), dot_q_data)
#        ddot_q_data = pr.save_part_log(degrees(ddot_q[0]), ddot_q_data)

    title_name = ['Link angle']
    label_name = ['q']
    xlabel_name = ['Time[s]', 'X[m]', 'θ-q[rad]']
    ylabel_name = ['Angle[deg]', 'Position[m]', 'Force [N]', 'Y[m]', 'K[Nm]']
    plt.figure(figsize=(6, 3))
    pr.print_graph(title_name[0], time_log, qdata,
                   label_name[0], xlabel_name[0], ylabel_name[0],
                   num_plot_data=1)

    x = ll[0] * cos(q[0])
    y = ll[0] * sin(q[0])

    r = sqrt(pow(ll[0]-x, 2) + pow(0-y, 2))

    L = 2 * ll[0] * 3.14 * degrees(q[0])/360
    print(r)
    print(L)
