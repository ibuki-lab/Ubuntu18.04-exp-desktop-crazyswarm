#!/usr/bin/env python
# coding: UTF-8

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from scipy.spatial import Voronoi
from shapely.geometry import Polygon


def main():
    # 有界領域を設定
    bnd = np.array([[-1.5, -1.5], [1.5, -1.5], [1.5, 1.5], [-1.5, 1.5]])

    # 母点の数
    # n = 100

    # 母点座標 ランダム
    # pnts = -np.random.rand(n, 2)
    # ゲインによっては最適解に収束しない例
    pnts = np.array([[1.35699453354, 1.06754016876],
                    [0.554793953896, 1.13054704666],
                    [-0.388975799084, 1.07037758827],
                    [0.618102908134, 0.498975843191],
                    [-0.389953583479, 0.535794377327],
                    [-1.3782794714, 0.535836517811],
                    [-1.40315451622, 1.00611484051]])
    print(pnts)
    # 計算回数
    time = range(100)

    # 動的なグラフを作成するための初期設定
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111)
    plt.ion()
    fig.show()
    fig.canvas.draw()

    # シミュレーション開始
    for t in time:

        # 現在の母点位置，ボロノイ領域，各ボロノイ領域の頂点座標

        gn_pnts, vor_polys, poly_vertice, pos_vertice = get_voronoi(bnd, pnts)

        # グラフの描画
        plot_figure(bnd, gn_pnts, vor_polys, fig, ax)

        # 各ボロノイ領域の重心計算

        centroid = get_centroid(poly_vertice, pos_vertice)

        # 各母点とそれに対応した重心の偏差を計算
        x_err, y_err = get_err(pnts, centroid)
        # 各母点の移動量を計算
        x_move, y_move = get_move(x_err, y_err)
        # 各母点の移動量を保存
        pnts = get_newpnts(pnts, x_move, y_move)
        print(pnts)


def get_voronoi(bnd, pnts):
    # bnd：ボロノイ分割する領域 範囲
    # pnts:母点座標
    """
    有界なボロノイ図を計算・描画する関数
    """

    # すべての母点のボロノイ領域を誘拐にするために，ダミー母点を三個追加
    # concatenate: 行列を縦方向に結合する
    gn_pnts = np.concatenate(
        [pnts, np.array([[100, 100], [100, -100], [-100, 0]])])

    # ボロノイ図の計算
    vor = Voronoi(gn_pnts)

    # 分割する領域をPolygonにする，有界領域
    bnd_poly = Polygon(bnd)
    # 各ボロノイ領域をしまうリスト
    vor_polys = []
    # 各ボロノイ領域のpolygonの頂点座標を格納
    poly_vertice = []
    # 各ボロノイ領域の平面座標系の頂点座標を格納
    pos_vertice = []
    for i in range(len(gn_pnts) - 3):

        # 平空間を考慮しないボロノイ領域
        """
        vertices: ボロノイ頂点座標を返す
        regions:すべてのボロノイ領域を格納している，
        region:ボロノイ領域
        point_region:ボロノイ領域の点が格納されている

        結局ボロノイ領域の各頂点座標を取得している
        """
        vor_poly = [vor.vertices[v] for v in vor.regions[vor.point_region[i]]]

        # 分割する領域をボロノイ領域の共通部分を計算
        """
        intersection: 積集合
        重複なしのボロノ頂点を取得
        """
        i_cell = bnd_poly.intersection(Polygon(vor_poly))
        pos_vertice.append(i_cell.exterior.coords[:-1])
        poly_vertice.append(Polygon(i_cell.exterior.coords[:-1]))

        # 平空間を考慮したボロノイ領域の頂点座標を格納
        vor_polys.append(list(i_cell.exterior.coords[:-1]))

        # 各母点の位置，ボロノイ図の領域，ボロノイ図の各頂点，ボロノイ図の各頂点(座標平面に適応済み)
    return gn_pnts, vor_polys, poly_vertice, pos_vertice


def plot_figure(bnd, gn_pnts, vor_polys, fig, ax):
    # ボロノイ図の描画
    ax.clear()
    xmin = np.min(bnd[:, 0])
    xmax = np.max(bnd[:, 0])
    ymin = np.min(bnd[:, 1])
    ymax = np.max(bnd[:, 1])
    ax.set_xlim(xmin-0.1, xmax+0.1)
    ax.set_ylim(ymin-0.1, ymax+0.1)
    ax.set_aspect('equal')
    # 母点の描画
    ax.scatter(gn_pnts[:-3][:, 0], gn_pnts[:-3][:, 1])
    # ボロノイ領域の描画
    """
    verts: 各ポリゴンの頂点を二次配列で渡す[[2, 2], [2, 3]]みたいに
    """
    poly_vor = PolyCollection(
        verts=vor_polys, edgecolor="black", facecolors="None", linewidth=1.0)
    ax.add_collection(poly_vor)
    fig.canvas.draw()
    fig.savefig("img2.png")


def get_centroid(poly_gem, pos_gem):
    # assert type(gem) == "shapely", "input should be a Shapely geometory"
    # assert gem.geom_type in ['Point', 'LineSrtring', 'Polygon'], "Input should be a Shapely geometry"
    # ポリゴンに変換
    # 重心を求める
    centroid = []
    for i in range(len(list(poly_gem))):
        centroid.append(poly_gem[i].centroid)

    # 重み付き重心を求めたいけど無理っぽい
    # centroid = []
    # centroid_x = 0
    # centroid_y = 0
    # for i in range(len(pos_gem)):
    #     for j in range(len(list(pos_gem[i]))):
    #         centroid_x += pos_gem[i][j][0]/len(pos_gem[i])* np.exp(-20 * (pos_gem[i][j][0] - 0.8)**2)
    #         centroid_y += pos_gem[i][j][1]/len(pos_gem[i])* np.exp(-20 * (pos_gem[i][1][1] - 0.6)**2)
    #         centroid.append(Polygon([centroid_x, centroid_y]))

    return centroid

# 求めた重心との偏差を計算する


def get_err(pnts, centroid):
    x_err = []
    y_err = []
    for i in range(len(pnts)):
        x_err.append(pnts[i][0] - np.array(centroid[i])
                     [0] + 0.0001 * np.random.randn())
        y_err.append(pnts[i][1] - np.array(centroid[i])
                     [1] + 0.0001 * np.random.randn())
    return x_err, y_err

# 求めた偏差から一度に動く量を計算する


def get_move(x_err, y_err):
    gain = 0.1
    x_move = []
    y_move = []
    for i in range(len(x_err)):
        x_move.append(-gain*x_err[i] + 0.00 * np.random.randn())
        y_move.append(-gain*y_err[i] + 0.00 * np.random.randn())
    return x_move, y_move

# 動いた後の位置を更新


def get_newpnts(pnts, x_move, y_move):
    for i in range(len(pnts)):
        # print(pnts, x_move)
        pnts[i][0] += x_move[i]
        pnts[i][1] += y_move[i]
    return pnts


if __name__ == "__main__":
    main()
