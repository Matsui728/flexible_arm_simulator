# -*- coding: utf-8 -*-
"""
Created on Fri Jan 26 15:13:12 2018

@author: kawalab
"""

import numpy as np
import matplotlib.pyplot as plt
import print_result as pr
import simlib as sl
from math import degrees, radians, sin, cos, atan2, sqrt, pow
from tqdm import tqdm


def Mechanical_friction(kp, Mm):
    omegan = kp
    K = pow(omegan, 2) * Mm

    return K

if __name__ == '__main__':
    Mm = 34.7*pow(10, -7)       # モータの慣性モーメント
    zeta = 1

    kp = 0.1
    kv = 0.000

    k1 = 10
    k2 = 2000
    k11 = sl.non_linear_parameta(k1, k2)
    k = [k11]

#    K = Mechanical_friction(kp, Mm)
    K = k1
    D = 2 * zeta * sqrt(Mm*kp)
    zeta1 = (D + kv) / 2 / sqrt(Mm*kp)

    omegan = sqrt(kp/Mm)

    print('kp = ' + str(kp))
    print('kv = ' + str(kv))
    print('D = ' + str(D))
    print('Omega_n = ' + str(omegan))
    print ('ζ = '  + str(zeta1))
