#!/user/bin/env python
# coding: UTF-8

import numpy as np
from scipy.spatial import Voronoi
from shapely.geometry import Polygon
from matplotlib.collections import PolyCollection


class Coverage_voronoi(object):

    def get_voronoi(self, bnd, pnts):
        # bnd：ボロノイ分割する領域 範囲
        # pnts:母点座標

        """
        有界なボロノイ図を計算・描画する関数
        """
        # self.fake_pnts = np.array([[100, 100], [100, -100], [-100, 0]])
        
        # すべての母点のボロノイ領域を誘拐にするために，ダミー母点を三個追加
        # concatenate: 行列を縦方向に結合する
        #self.gn_pnts = np.concatenate(pnts,fake_pnts)
        
        # ボロノイ図の計算
        self.vor = Voronoi(pnts)

        # 分割する領域をPolygonにする，有界領域
        self.bnd_poly = Polygon(bnd)
        # 各ボロノイ領域をしまうリスト
        self.vor_polys = []
        # 各ボロノイ領域のpolygonの頂点座標を格納
        self.poly_vertice = []
        # 各ボロノイ領域の平面座標系の頂点座標を格納
        self.pos_vertice = []

        for i in range(len(pnts) - 3):
            
            """
            vertices: ボロノイ頂点座標を返す
            regions:すべてのボロノイ領域を格納している，
            region:ボロノイ領域
            point_region:ボロノイ領域の点が格納されている

            結局ボロノイ領域の各頂点座標を取得している
            """
            self.vor_poly = [self.vor.vertices[v] for v in self.vor.regions[self.vor.point_region[i]]]


            # 分割する領域をボロノイ領域の共通部分を計算
            """
            intersection: 積集合
            重複なしのボロノ頂点を取得
            """
            i_cell = self.bnd_poly.intersection(Polygon(self.vor_poly))
            self.pos_vertice.append(i_cell.exterior.coords[:-1])
            self.poly_vertice.append(Polygon(i_cell.exterior.coords[:-1]))

            # 平空間を考慮したボロノイ領域の頂点座標を格納
            self.vor_polys.append(list(i_cell.exterior.coords[:-1]))

        # 各母点の位置，ボロノイ図の領域，ボロノイ図の各頂点，ボロノイ図の各頂点(座標平面に適応済み)
        return  self.vor_polys, self.poly_vertice, self.pos_vertice
###############################################################################################################

    def get_centroid(self, poly_gem, pos_gem):

        # 重心を求める

        centroid = []
        for i in range(len(list(poly_gem))):
            centroid.append(poly_gem[i].centroid)
        
        return centroid

    # 求めた重心との偏差を計算する
    def get_err(self, pnts, centroid):
        x_err = []
        y_err = []
        for i in range(len(pnts)-3):
            x_err.append(pnts[i][0] - np.array(centroid[i])[0] + 0.0001 * np.random.randn())
            y_err.append(pnts[i][1] - np.array(centroid[i])[1] + 0.0001 * np.random.randn())
        return x_err, y_err

    # 求めた偏差から一度に動く量を計算する
    def get_move(self, x_err, y_err):
        gain=0.1
        x_move = []
        y_move = []
        for i in range(len(x_err)):
            x_move.append(-gain*x_err[i] + 0.00* np.random.randn())
            y_move.append(-gain*y_err[i] + 0.00 * np.random.randn())
        return x_move, y_move 

    # 動いた後の位置を更新
    def get_newpnts(self, pnts, x_move, y_move):
        for i in range(len(pnts)-3):
            pnts[i][0] += x_move[i]
            pnts[i][1] += y_move[i]
        print(pnts)
        return pnts

    # ボロノイ図の描画
    def plot_figure(self, bnd, gn_pnts, vor_polys, fig, ax):

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
        poly_vor = PolyCollection(verts=vor_polys, edgecolor="black", facecolors="None", linewidth=1.0)
        ax.add_collection(poly_vor)
        fig.canvas.draw()
        fig.savefig("img.png")
    
#######################################################################################################
