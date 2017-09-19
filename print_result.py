# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 10:56:16 2017

@author: kawalab
"""

import numpy as np
import matplotlib.pyplot as plt
from math import degrees


def print_graph(title_name, x_data, y_data,
                label_name, xlabel_name, ylabel_name, num_plot_data=1):
    plt.title(title_name, fontsize=15, fontname='Times New Roman')

    for i in range(num_plot_data):
        plt.plot(x_data, y_data[i-1], label=label_name[i-1])

    plt.xlabel(xlabel_name, fontsize=15, fontname='Times New Roman')  # x軸のタイトル
    plt.ylabel(ylabel_name, fontsize=15, fontname='Times New Roman')  # y軸のタイトル
    plt.legend(bbox_to_anchor=(1.01, 1), loc=2, borderaxespad=0)
    plt.grid()
    plt.tight_layout()


def save_angle_excel_log(time, q, qd, dot_q, ddot_q):
    log_data = "{}, {}, {}, {}, {}, {}, {}, {}, {},"
    + "{}, {}, {}, {},"
    + "{}, {}, {}, {}\n". format(time, degrees(q[0]), degrees(q[1]),
                                 degrees(q[2]), degrees(q[3]),
                                 degrees(qd[0]), degrees(qd[1]),
                                 degrees(qd[2]), degrees(qd[3]),
                                 dot_q[0], dot_q[1], dot_q[2], dot_q[3],
                                 ddot_q[0], ddot_q[1], ddot_q[2], ddot_q[3])

    return log_data


def save_position_log(time, x_data, y_data, xd_data, yd_data,
                      lambdax_data, lambday_data):
    log_data = "{}, {}, {}, {}, {}, {}\n". format(time, x_data, y_data,
                                                  xd_data, yd_data,
                                                  lambdax_data, lambday_data)

    return log_data



def save_part_log(x_data, x_data_log):
    x_data_log.append(x_data)

    return x_data_log



def make_data_log_list(data_list):
    out_put_data_set = []
    for i in range(len(data_list)):
        data_log = save_part_log(data_list[i-1], data_list[i-1])
        out_put_data_set.append(data_log)

    return out_put_data_set


if __name__ == '__main__':
    print_graph()
