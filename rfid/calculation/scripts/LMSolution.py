#!/usr/bin/python
# -*- coding: UTF-8 -*-

import numpy as np
import sys
from numpy import matrix as mat
import random
import function as func
import matplotlib as mpl
import matplotlib.pyplot as plt

# 载入数据
LengthRestrict = np.array([-0.2,0.2])
WidthRestrict = np.array([0.4,0.8])
HeightRestrict = np.array([0.2,0.6])

# 输入标签实际坐标
TagReal = np.array([0,0.6,0.265])

# 天线读取的坐标
AntennaReal= np.array([
[-0.2,    0.4,    0.6],  
[-0.2,  0.425,    0.6],
[-0.2,   0.45,    0.6],
[-0.2,  0.475,    0.6],
[-0.2,    0.5,    0.6],
[-0.2,  0.525,    0.6],
[-0.2,   0.55,    0.6],
[-0.2,  0.575,    0.6],
[-0.2,    0.6,    0.6],
[-0.175,    0.6,    0.6],
[-0.15,    0.6,    0.6],
[-0.125,    0.6,    0.6],
[-0.1,    0.6,    0.6],
[-0.075,    0.6,    0.6],
[-0.05,    0.6,    0.6],
[-0.025,    0.6,    0.6],
[0,    0.6,    0.6],
[0.025,    0.6,    0.6],
[0.05,    0.6,    0.6],
[0.075,    0.6,    0.6],
[0.1,    0.6,    0.6],
[0.125,    0.6,    0.6],
[0.15,    0.6,    0.6],
[0.175,    0.6,    0.6],
[0.2,    0.6,    0.6],
[0.2,    0.6,  0.575],
[0.2,    0.6,   0.55],
[0.2,    0.6,  0.525],
[0.2,    0.6,    0.5],
[0.2,    0.6,  0.475],
[0.2,    0.6,   0.45],
[0.2,    0.6,  0.425]])

# 导入读取到的相位值
Phase = np.array([ 10.540114,
                   30.430439,
                   45.737592,
                   62.681526,
                   75.193015,
                   86.807812,
                   95.346680,
                  101.100399,
                  105.871582,
                  130.341797,
                  333.738281,
                  355.651325,
                   15.011719,
                  210.911458,
                  222.862120,
                  229.104492,
                  233.203125,
                  232.711694,
                  228.527344,
                  220.087891,
                   28.125000,
                   13.945312,
                  358.056066,
                  340.255490,
                  139.306641,
                    5.395408,
                  230.337358,
                  270.832270,
                  310.160846,
                  350.764724,
                   31.930147,
                   70.150240])

# 数据预处理
# 相位解缠
phaseUnwrapped = func.unwrappingPhase(Phase)

List = np.array(range(Phase.shape[0]))
'''
plt.figure()
plt.scatter(List,Phase)
plt.figure()
plt.scatter(List,phaseUnwrapped)
plt.show()
'''

# LM算法求解
location,error = func.LMConduct(phaseUnwrapped,AntennaReal,0.1,0.5,0.3)
print("The true position is:",TagReal)
print("The predicted position is:",location)
print("error:",error)

