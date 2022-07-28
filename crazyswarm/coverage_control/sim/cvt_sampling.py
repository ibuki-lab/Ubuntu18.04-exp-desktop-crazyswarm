#! /usr/bin/env python3
def cvt_2d_sampling ( g_num = 0, it_num = 0, s1d_num = 0 ):

#  Input:
#
#    integer G_NUM, the number of generators.
#    A value of 50 is reasonable.
#
#    integer IT_NUM, the number of CVT iterations.
#    A value of 20 or 50 might be reasonable.
#
#    integer S1D_NUM, the number of sample points to use
#    when estimating the Voronoi regions.
#    A value of 1,000 is too low.  A value of 1,000,000 is somewhat high.
#
  import numpy as np
  import matplotlib.pyplot as plt
  import platform
  from scipy.spatial import Delaunay
  from scipy.spatial import Voronoi
  from scipy.spatial import voronoi_plot_2d
  from scipy import argmin
  from scipy import inner

  if ( g_num <= 0 ):
    prompt = '  Enter number of generators:  '
    inp = input ( prompt )
    g_num = int ( inp )

  if ( it_num <= 0 ):
    prompt = '  Enter number of iterations:  '
    inp = input ( prompt )
    it_num = int ( inp )

  if ( s1d_num <= 0 ):
    prompt = '  Enter number of sample points:  '
    inp = input ( prompt )
    s1d_num = int ( inp )

  s_num = s1d_num * s1d_num

  print ( '' )
  print ( '  Number of generators is %d' % ( g_num ) )
  print ( '  Number of iterations is %d' % ( it_num ) )
  print ( '  Number of 1d samples is %d' % ( s1d_num ) )
  print ( '  Number of 2d samples is %d' % ( s_num ) )
#
# ジェネレータを初期化します。
#
  gx = np.random.rand ( g_num )
  gy = np.random.rand ( g_num )
#
# サンプルポイントの固定グリッドを生成します。
# MESHGRIDのドキュメントはあいまいすぎるので、難しい方法でやってみましょう。
#
  s_1d = np.linspace ( 0.0, 1.0, s1d_num )
  sx = np.zeros ( s_num )
  sy = np.zeros ( s_num )
  s = np.zeros ( [ s_num, 2 ] )
  k = 0
  for j in range ( 0, s1d_num ):
    for i in range ( 0, s1d_num ):
      sx[k] = s_1d[i]
      sy[k] = s_1d[j]
      s[k,0] = s_1d[i]
      s[k,1] = s_1d[j]
      k = k + 1
#
#反復を実行します。
#
# エラー：すべてのステップでログ（E）とログ（GM）をプロットしたい。
# EとGMをNANに初期化する必要があります。
# MATLAB、プロットしません。
#
  step = np.zeros ( it_num )
  e = 1.0E-10 * np.ones ( it_num )
  gm = 1.0E-10 * np.ones ( it_num )

  for it in range ( 0, it_num ):

    step[it] = it
#
#  現在のノードのドロネー三角形情報Tを計算します。
#  私のバージョンのNUMPYが古すぎるため、STACKコマンドを使用できません
#  2つのベクトルを1つの配列に結合します。
#
    g = np.zeros ( [ g_num, 2 ] )
    g[:,0] = gx[:]
    g[:,1] = gy[:]
    tri = Delaunay ( g )
#
# ボロノイセルを表示します。
#
    subfig1 = plt.subplot ( 2, 2, 1 )   
    vor = Voronoi ( g )
    voronoi_plot_2d ( vor, ax = subfig1 )
#
# ドロネー三角形分割を表示します。
#
    subfig2 = plt.subplot ( 2, 2, 2 )
    plt.triplot ( gx, gy, tri.simplices.copy( ) )
    plt.plot ( gx, gy, 'o' )
#
#  各サンプルポイントについて、最も近いジェネレーターのインデックスであるKを見つけます。
#
    k = [ argmin ( [ inner ( gg - ss, gg - ss ) for gg in g ] ) for ss in s ]
#
#  各ジェネレーターについて、Mは最も近いサンプルポイントの数をカウントします。
#  密度が不均一な場合は、Wを密度に設定するだけであることに注意してください。
    w = np.ones ( s_num )
    m = np.bincount ( k, weights = w )
#
#  Gは、最も近いサンプルポイントの平均です。
#  Mがゼロの場合、Gを変更しないでください。
#
    gx_new = np.bincount ( k, weights = sx )
    gy_new = np.bincount ( k, weights = sy )

    for i in range ( 0, g_num ):
      if ( 0 < m[i] ):
        gx_new[i] = gx_new[i] / float ( m[i] )
        gy_new[i] = gy_new[i] / float ( m[i] )
        
##################################################################
#
# エネルギーを計算します。
#
#    e[it] = 0.0
#    for i in range ( 0, s_num ):
#      e[it] = e[it] + ( sx[i] - gx_new[k[i]] ) ** 2 \
#                    + ( sy[i] - gy_new[k[i]] ) ** 2
#
## エネルギーのログを表示します。
#
#    subfig3 = plt.subplot ( 2, 2, 3 )
#    plt.plot ( step[0:it+1], np.log ( e[0:it+1] ), 'm-*', linewidth = 2 )
#    plt.title ( 'Log (Energy)' )
#    plt.xlabel ( 'Step' )
#    plt.ylabel ( 'Energy' )
#    plt.grid ( True )
#
## 発電機の動きを計算します。 
#
#    for i in range ( 0, g_num ):
#      gm[it] = gm[it] + ( gx_new[i] - gx[i] ) ** 2 \
#                     + ( gy_new[i] - gy[i] ) ** 2
#
## 発電機の動きを表示します。
#
#    subfig4 = plt.subplot ( 2, 2, 4 )
#    plt.plot ( step[0:it+1], np.log ( gm[0:it+1] ), 'm-*', linewidth = 2 )
#    plt.title ( 'Log (Average generator motion)' )
#    plt.xlabel ( 'Step' )
#    plt.ylabel ( 'Energy' )
#    plt.grid ( True )
##################################################################
#
# プロットにタイトルを付けます。
#
    super_title = 'Iteration ' + str ( it )
    plt.suptitle ( super_title )
#
# 最初と最後のプロットを保存します。
#
    if ( it == 0 ):
      filename = 'initial.png'
      plt.savefig ( filename )
      print ( '  Graphics saved as "', filename, '"' )
    elif ( it == it_num - 1 ):
      filename = 'final.png'
      plt.savefig ( filename )
      print ( '  Graphics saved as "', filename, '"' )

    plt.show ( block = False )
    plt.close ( )
#
# ジェネレーターを更新します。
#
    gx = gx_new
    gy = gy_new

  return

def cvt_2d_sampling_test ( ):

  cvt_2d_sampling ( 16, 20, 100 )

  return


if ( __name__ == '__main__' ):
  cvt_2d_sampling_test ( )
  
