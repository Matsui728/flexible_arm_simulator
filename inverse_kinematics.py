# -*- coding: utf-8 -*-
"""
Created on Thu Nov  2 15:05:52 2017

@author: kawalab
"""

import numpy as np
import matplotlib.pyplot as plt
from math import degrees, radians, sin, cos
from math import atan2, pow, sqrt

def cul_inverse_kinematics_3dof(x, y, l, theta):

    a = y - l[2] * sin(theta)      # θはベース-手先間の角度
    b = x - l[2] * cos(theta)
    c = (pow((y - l[2] * sin(theta)), 2) + pow(x - l[2] * cos(theta), 2) + pow(l[0], 2) - pow(l[1], 2))/ (2 * l[0])
    d = (pow((y - l[2] * sin(theta)), 2) + pow(x - l[2] * cos(theta), 2) - pow(l[0], 2) + pow(l[1], 2))/ (2 * l[1])

    theta1 = degrees(atan2(a, b) - atan2(-sqrt(pow(a, 2) + pow(b, 2) - pow(c, 2)), c))

    theta2 = degrees(atan2(-sqrt(pow(a, 2) + pow(b, 2) - pow(c, 2)), c) - atan2(sqrt(pow(a, 2) + pow(b, 2) - pow(d, 2)), d))

    theta3 = degrees(theta) - theta1 - theta2

    thetas = [theta1, theta2, theta3]

    return thetas


def cul_inverse_kinematics_2dof(x, y, l):
    a = (pow(x, 2) + pow(y, 2) + pow(l[0], 2) - pow(l[1], 2)) / (2 * l[0])
    b = (pow(x, 2) + pow(y, 2) - pow(l[0], 2) + pow(l[1], 2)) / (2 * l[1])

    c = pow(x, 2) + pow(y, 2) - pow(a, 2)
    d = pow(x, 2) + pow(y, 2) - pow(b, 2)

    theta1 = atan2(y, x) + atan2(sqrt(c), a)
    theta2 = -atan2((sqrt(c)), a) - atan2(sqrt(d), b)

    thetas = [degrees(theta1), degrees(theta2)]

    return thetas


def make_intial_angle(x, y, l1, l2, theta0):
    q1 = cul_inverse_kinematics_3dof(x, y, l1, theta0)
    q2 = cul_inverse_kinematics_2dof(x, y, l2)

    theta1 = cul_inverse_kinematics_3dof(x, y, l1, theta0)
    theta2 = cul_inverse_kinematics_2dof(x, y, l2)

    q = [radians(q1[0]), radians(q1[1]), radians(q1[2]),
         radians(q2[0]), radians(q2[1])]
    theta = [radians(theta1[0]), radians(theta1[1]), radians(theta1[2]),
             radians(theta2[0]), radians(theta2[1])]

    return q, theta


def make_intial_angle_2dof(x1, y1, x2, y2, l1, l2):
    q1 = cul_inverse_kinematics_2dof(x1, y1, l1)
    q2 = cul_inverse_kinematics_2dof(x2, y2, l2)
    theta1 = cul_inverse_kinematics_2dof(x1, y1, l1)
    theta2 = cul_inverse_kinematics_2dof(x2, y2, l2)

    q = [radians(q1[0]), radians(q1[1]),
         radians(q2[0]), radians(q2[1])]
    theta = [radians(theta1[0]), radians(theta1[1]),
             radians(theta2[0]), radians(theta2[1])]

    return q, theta


def plot_arm(l1, l2, theta1, theta2):
    x1 = [0, l1[0]*cos(radians(theta1[0])),
          l1[0]*cos(radians(theta1[0])) + l1[1]*cos((radians(theta1[0]+theta1[1]))),
          l1[0]*cos(radians(theta1[0])) + l1[1]*cos((radians(theta1[0]+theta1[1]))) + l1[2]*cos(radians(theta1[0]+theta1[1]+theta1[2]))]
    y1 = [0, l1[0]*sin(radians(theta1[0])),
          l1[0]*sin(radians(theta1[0])) + l1[1]*sin((radians(theta1[0]+theta1[1]))),
          l1[0]*sin(radians(theta1[0])) + l1[1]*sin((radians(theta1[0]+theta1[1]))) + l1[2]*sin(radians(theta1[0]+theta1[1]+theta1[2]))]

    x2 = [0, l2[0]*cos(radians(theta2[0])),
          l2[0]*cos(radians(theta2[0])) + l2[1]*cos((radians(theta2[0]+theta2[1])))]

    y2 = [0, l2[0]*sin(radians(theta2[0])),
          l2[0]*sin(radians(theta2[0])) + l2[1]*sin((radians(theta2[0]+theta2[1])))]

    x0 = [0, x2[2]]
    y0 = [0, y2[2]]

    plt.figure(figsize=(5, 5))
    plt.plot(x0, y0, "-c", lw=3, label="Radial straight line")
    plt.plot(x2, y2, "-b", lw=3, label="arm2")
    plt.plot(x2, y2, "ob", lw=5, ms=5, label="joint2")
    plt.plot(x1, y1, "-r", lw=3, label="arm1")
    plt.plot(x1, y1, "or", lw=5, ms=5, label="joint1")
#    plt.legend(bbox_to_anchor=(1.01, 1), loc=2, borderaxespad=0)
    plt.xlim(-0.7, 0.7)
    plt.ylim(-0.7, 0.7)

    plt.grid()
    plt.show()


if __name__ == '__main__':

    l1 = [0.3, 0.3, 0.1]
    l2 = [0.35, 0.35]
    x = 0.0
    y = 0.4
    theta = atan2(y, x) - radians(90)

    thetas1 = cul_inverse_kinematics_3dof(x, y, l1, theta)
    thetas2 = cul_inverse_kinematics_2dof(x, y, l2)
    plot_arm(l1, l2, thetas1, thetas2)

    print('{}, {}, {}\n'. format(thetas1[0], thetas1[1], thetas1[2]))
    print('{}, {}\n'. format(thetas2[0], thetas2[1]))
