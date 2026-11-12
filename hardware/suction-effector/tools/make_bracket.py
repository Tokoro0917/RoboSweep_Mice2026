"""
吸着ユニットをサーボ2に取り付ける金具（コの字形）を作るスクリプト
（hardware/suction-effector で実行）
    pip install trimesh manifold3d numpy shapely
    python tools/make_bracket.py

考え方
  - サーボ2の両側のホーン（RH01 とフリーホーン）を左右から挟み、M2 で留める。
  - 下側の板に、吸着ユニットのアーム（M3 × 2、間隔 16mm）を下から当てて留める。
  - ボールを吸うとき、吸着ユニットは「後ろ側を 15° 下げた」向き（案B）になる。
    そのときの腕の角度は約 25°、ボールの中心は機体の前の端から約 19mm 前。

寸法は機体全体の STEP（全体 v11）から測った値。
  サーボ1の軸 (y, z) = (25.73, 67.22)、サーボ2の軸までの長さ 39.52mm
  タイヤの下端（床）z = -3.4
  ホーンの外側の面 x = +11.4（RH01）/ -17.7（フリーホーン）
  ホーンのねじ穴: 半径 6.22mm の円周上（RH01 は 45° から 90° ごとに 4 つ）

できるもの
  robosweep_bracket.stl      : 印刷用（ねじ板を下にする。サポートなし）
  robosweep_bracket_pose.stl : 機体の座標に置いたもの（確認用）
"""
import math

import numpy as np
import trimesh
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union
from trimesh.creation import cylinder, extrude_polygon

# ---- 機体の寸法（STEP から） ----
FLOOR = -3.4
J1 = np.array([25.73, 67.22])          # サーボ1の軸 (y, z)
ARM_L = 39.52                          # サーボ1の軸 → サーボ2の軸
HORN_FACE_R = 11.4                     # RH01 の外側の面（x）
HORN_FACE_L = -17.7                    # フリーホーンの外側の面（x）
HORN_HOLE_R = 6.22                     # ホーンのねじ穴の半径

# ---- 吸着ユニット（robosweep_effector_assembled.stl） ----
BALL_R = 35.0
SEAT_Z = 32.0                          # 吸着座（コーンの口から）
TAB_TOP_Z = 48.8                       # 取り付けアームの上面
BOLT_IN, BOLT_OUT = 26.62, 42.62       # アームのねじ穴の、ユニットの軸からの距離
EFFECTOR_X = (HORN_FACE_R + HORN_FACE_L) / 2   # ユニットの左右の中心 = 左右のホーンの真ん中

# ---- 金具の形 ----
TILT = -15.0                           # 吸着ユニットの傾き（- で後ろ側が下がる）
DROP = 16.0                            # サーボ2の軸 → アームの上面
FOOT_V = 8.0                           # サーボ2の軸の真下の位置（内側のねじ穴から後ろ向きに測る）。0 だとサーボ1に当たる
PLATE_T = 2.5                          # 左右の板の厚さ
BASE_T = 3.0                           # ねじ板の厚さ
BAR = (-5.0, 5.0)                      # 左右の板をつなぐ横板の範囲（サーボ2の軸の真下から、後ろ向きを +）
TONGUE_V = (-4.5, 20.0)                # アームを留める細い板の範囲（内側のねじ穴を 0）。前の端は天井（半径 20.6mm）に当たらないように
TONGUE_W = 14.0                        # 細い板の幅（アームは 10mm）
HORN_DISC_R = 9.0
M2_HOLE = 2.2
CENTER_HOLE = 6.5                      # ホーンの中心のねじの頭を逃がす
M3_HOLE = 3.4


def pose():
    """ボールを吸っている姿勢を求める。ボールの前後の位置を動かして、サーボ2の軸が腕の長さに来る所を探す"""
    t = math.radians(TILT)
    n = np.array([math.sin(t), math.cos(t)])        # ユニットの軸（上向き）
    side = np.array([math.cos(t), -math.sin(t)])    # 前向き
    back = -side
    seat_off = math.sqrt(BALL_R**2 - 17.5**2)
    best = None
    for yb in np.arange(80.0, 200.0, 0.01):
        seat = np.array([yb, FLOOR + BALL_R]) + seat_off * n
        top = seat + (TAB_TOP_Z - SEAT_Z) * n
        inner = top + BOLT_IN * back
        j2 = inner + FOOT_V * back + DROP * n
        err = abs(np.linalg.norm(j2 - J1) - ARM_L)
        if best is None or err < best[0]:
            best = (err, yb, j2, inner, n, back, seat)
    return best


def to3d(mesh_yzx):
    """(y, z, x) で作った形を (x, y, z) に並べ替える"""
    m = mesh_yzx.copy()
    T = np.zeros((4, 4)); T[3, 3] = 1
    T[0, 2] = 1; T[1, 0] = 1; T[2, 1] = 1
    m.apply_transform(T)
    return m


def build():
    err, yb, j2, inner, n, back, seat = pose()
    angle = math.degrees(math.atan2(j2[1] - J1[1], j2[0] - J1[0]))
    print(f"ボールの中心 y={yb:.1f}mm  腕の角度 {angle:.1f}°  サーボ2の軸 ({j2[0]:.1f}, {j2[1]:.1f})")

    def q(v, w):
        return inner + v * back + w * n

    bar_poly = Polygon([q(FOOT_V + BAR[0], 0), q(FOOT_V + BAR[1], 0),
                        q(FOOT_V + BAR[1], BASE_T), q(FOOT_V + BAR[0], BASE_T)])
    tongue_poly = Polygon([q(TONGUE_V[0], 0), q(TONGUE_V[1], 0), q(TONGUE_V[1], BASE_T), q(TONGUE_V[0], BASE_T)])
    disc = Point(*j2).buffer(HORN_DISC_R, 128)
    plate_poly = unary_union([disc, bar_poly]).convex_hull

    parts = []
    xl, xr = HORN_FACE_L - PLATE_T, HORN_FACE_R + PLATE_T
    for x0, x1 in ((HORN_FACE_R, xr), (xl, HORN_FACE_L)):
        p = extrude_polygon(plate_poly, x1 - x0)
        p.apply_translation([0, 0, x0])
        parts.append(to3d(p))
    b = extrude_polygon(bar_poly, xr - xl)
    b.apply_translation([0, 0, xl])
    parts.append(to3d(b))
    t = extrude_polygon(tongue_poly, TONGUE_W)
    t.apply_translation([0, 0, EFFECTOR_X - TONGUE_W / 2])
    parts.append(to3d(t))
    bracket = trimesh.boolean.union(parts, engine="manifold")

    holes = []
    span = 4 * PLATE_T + abs(HORN_FACE_R - HORN_FACE_L) + 10
    xmid = (HORN_FACE_R + HORN_FACE_L) / 2
    def x_cyl(d, y, z):
        c = cylinder(radius=d / 2, height=span, sections=48)
        c.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
        c.apply_translation([xmid, y, z])
        return c
    holes.append(x_cyl(CENTER_HOLE, *j2))
    for k in range(4):
        a = math.radians(45 + 90 * k)
        holes.append(x_cyl(M2_HOLE, j2[0] + HORN_HOLE_R * math.cos(a), j2[1] + HORN_HOLE_R * math.sin(a)))
    for v in (0.0, BOLT_OUT - BOLT_IN):
        c = cylinder(radius=M3_HOLE / 2, height=20, sections=48)
        ang = math.atan2(n[0], n[1])                 # z 軸を n の向きへ
        c.apply_transform(trimesh.transformations.rotation_matrix(-ang, [1, 0, 0]))
        p = q(v, 0)
        c.apply_translation([EFFECTOR_X, p[0], p[1]])
        holes.append(c)
    bracket = trimesh.boolean.difference([bracket] + holes, engine="manifold")
    return bracket, (yb, j2, inner, n, back, seat, angle)


def main():
    bracket, (yb, j2, inner, n, back, seat, angle) = build()
    bracket.export("robosweep_bracket_pose.stl")

    # 印刷の向き: ねじ板の下面（アームに当たる面）を台に置く
    m = bracket.copy()
    z = np.array([0.0, -n[0], -n[1]])               # 下面の外向き → -Z
    ang = math.atan2(n[0], n[1])
    m.apply_transform(trimesh.transformations.rotation_matrix(ang, [1, 0, 0]))
    m.apply_translation(-m.bounds[0])
    m.export("robosweep_bracket.stl")
    print("robosweep_bracket.stl: 閉じた形", m.is_volume, "大きさ", (m.bounds[1] - m.bounds[0]).round(1).tolist(),
          "体積 %.2fcm3" % (m.volume / 1000))


if __name__ == "__main__":
    main()
