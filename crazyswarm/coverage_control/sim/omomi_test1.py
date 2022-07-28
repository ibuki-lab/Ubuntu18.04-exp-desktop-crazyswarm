import random
import matplotlib.pyplot as plt

import numpy as np
from numpy import inner
from numpy.linalg import norm

def angle2(P, A, B):
    '''
    args
    ----
        P: np.array((x, y))
        A: np.array((x, y))
        B: np.array((x, y))
    returns
    -------
        theta: float (dgree)
    '''
    # 全体をシフトさせる
    PA = A - P
    PB = B - P
    # cosθを求める
    costheta = inner(PA, PB)/norm(PA)/norm(PB)
    # 角度を求める(度)
    theta = np.arccos(costheta)/np.pi * 180
    # 時計回りか、反時計回りか判定し、thetaをプラスかマイナスにして返す
    if PA[0]*PB[1] - PA[1]*PB[0] > 0:
        # 条件(i)'に該当するときはなにもしない(ここではthetaはプラス)
        pass
    else:
        # 条件(ii)'に該当するときはthetaをマイナスにする
        theta *= -1.0
    return theta

def is_inside_polygon(points, P):
    '''
    args
    ----
        points: list of np.array((x, y))
        P: np.array((x, y))
    returns
    -------
        theta: x (degree)
    '''
    points = [np.array(p) for p in points]
    P = np.array(P)
    angle_total = 0.0
    for A, B in zip(points, points[1:] + points[0:1]): # Notice! "points[1:] + points[0:1]" means something like "points[1:].extend(points[0:1])"
        angle_total += angle2(P, A, B)
    if np.isclose(abs(angle_total), 360):
        # 360°の場合はポリゴンの内側なのでTrueを返す
        return True
    else:
        # ポリゴンの外側
        return False

# 三角形
#points = ((0,0), (1,0), (0, 1))

# リボン?
#points = ((1.0, 0.0), (0.5, 0.5), (1.0, 1.0), (-1.0, 1.0), (-0.5, 0.5), (-1.0, 0.0))

# 十字
points = ((1.0, -3.0), (1.0, -1.0), (3.0, -1.0), (3.0, 1.0), (1.0, 1.0), (1.0, 3.0), (-1.0, 3.0), (-1.0, 1.0), (-3.0, 1.0), (-3.0, -1.0), (-1.0, -1.0), (-1.0, -3.0))
points = [(i[0]/3, i[1]/3) for i in points]

N = 5000
for _ in range(N):
    # 点を打つ場所をランダムに決める(ただし、-1<x<1, -1<y<1とする)
    x = (random.random() - 0.5) * 2
    y = (random.random() - 0.5) * 2
    p = (x, y)
    # 決めた点(x, y)が三角形の内側にあればグラフに点を打つ
    if is_inside_polygon(points, p):
        plt.plot(x, y, color='k', marker='.')

# グラフを表示する
plt.gca().set_xlim(-2,2)
plt.gca().set_ylim(-2,2)
plt.gca().set_aspect('equal')
plt.show()