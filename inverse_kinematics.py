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

    c = x*x + y*y - a*a
    d = x*x + y*y - b*b

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


if __name__ == '__main__':
    theta = radians(45)
    l1 = [0.3, 0.3, 0.1]
    l2 = [0.35, 0.35]
    x = 0.3
    y = 0.3

    thetas1 = cul_inverse_kinematics_3dof(x, y, l1, theta)
    thetas2 = cul_inverse_kinematics_2dof(x, y, l2)

    print('{}, {}, {}\n'. format(thetas1[0], thetas1[1], thetas1[2]))
    print('{}, {}\n'. format(thetas2[0], thetas2[1]))
