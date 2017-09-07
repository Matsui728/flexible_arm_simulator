# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 10:56:16 2017

@author: kawalab
"""

import numpy as np
import matplotlib.pyplot as plt


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


if __name__ == '__main__':
    print_graph()
