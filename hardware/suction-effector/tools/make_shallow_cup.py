"""
吸着ユニットの下の部品（コーン）を浅いカップにした形を作るスクリプト（hardware/suction-effector で実行）
    pip install trimesh manifold3d numpy
    python tools/make_shallow_cup.py

今の下の部品（robosweep_effector_lower.stl）のうち、上の部品とはまる管・吸込口の板・ボールが当たる座
（印刷の向きで z = 0〜7mm）はそのまま使い、そこから先のコーンを作り直す。
座の位置は変わらないので、取り付け金具・アームの角度・プログラムは今のままでよい。

印刷の向き（今の下の部品と同じ。上の部品とはまる側が下、口が上）での座標:
  ボールの中心 z = 36.3mm（座の輪 Φ35 にボールが当たったとき）

できるもの
  robosweep_effector_lower_cup1.stl : 案 1 浅いカップ（ボールの上 1/3 を包む。口 約 Φ80）
  robosweep_effector_lower_cup2.stl : 案 2 浅いカップ + 広い縁（口 約 Φ92）
"""
import math

import numpy as np
import trimesh
from trimesh.creation import box, revolve

BALL_R = 35.0
BALL_Z = 36.3        # ボールの中心（印刷の向き）
GAP = 1.0            # カップの内面とボールのすき間
WALL = 1.2           # カップの壁の厚さ（今のコーンと同じくらい。ノズル 0.4mm で 3 本）
KEEP_Z = 7.0         # 今の部品のうち、残す高さ（管・吸込口の板・座）
FLARE_ANGLE = 45.0   # 縁の広がりの角度（水平から）。45° より寝かせるとサポートが要る

# (名前, ボールを包む角度 [deg]（真上から測る）, 縁の内径の半分 [mm])
VARIANTS = [
    ("robosweep_effector_lower_cup1.stl", 50.0, 38.0),
    ("robosweep_effector_lower_cup2.stl", 50.0, 44.0),
]


def inner_line(cover_deg, rim_r):
    """カップの内面（r, z）。座（30°）からボールに沿って cover_deg まで、そこから縁を広げる"""
    pts = []
    for ph in np.linspace(28.0, cover_deg, 40):
        a = math.radians(ph)
        pts.append(((BALL_R + GAP) * math.sin(a), BALL_Z - (BALL_R + GAP) * math.cos(a)))
    r0, z0 = pts[-1]
    pts.append((rim_r, z0 + (rim_r - r0) * math.tan(math.radians(FLARE_ANGLE))))
    return np.array(pts)


def offset_out(line, d):
    """線を外側（r が増える側、ボールから離れる側）に d だけずらす"""
    out = []
    n = len(line)
    for i in range(n):
        p0 = line[max(i - 1, 0)]
        p1 = line[min(i + 1, n - 1)]
        t = p1 - p0
        t = t / np.linalg.norm(t)
        nrm = np.array([t[1], -t[0]])   # 進む向きの右（r が増える向き）
        if nrm[0] < 0:
            nrm = -nrm
        out.append(line[i] + d * nrm)
    return np.array(out)


def build(cover_deg, rim_r):
    base = trimesh.load("robosweep_effector_lower.stl")
    keep = box(extents=[200, 200, KEEP_Z])
    keep.apply_translation([0, 0, KEEP_Z / 2])
    base = trimesh.boolean.intersection([base, keep], engine="manifold")

    inner = inner_line(cover_deg, rim_r)
    outer = offset_out(inner, WALL)
    prof = np.vstack([inner, outer[::-1], inner[:1]])
    wall = revolve(prof, sections=192)
    if wall.volume < 0:
        wall.invert()
    # 下端（座のあたり）は今の部品に重ねてつなぐ
    m = trimesh.boolean.union([base, wall], engine="manifold")
    m.merge_vertices(digits_vertex=4)
    m.update_faces(m.nondegenerate_faces(height=1e-6))
    m.remove_unreferenced_vertices()
    return m, inner


def main():
    for name, cover, rim in VARIANTS:
        m, inner = build(cover, rim)
        m.export(name)
        ok = trimesh.load(name).is_volume
        top_r, top_z = inner[-1]
        dz = BALL_Z - top_z
        catch = top_r - math.sqrt(max(BALL_R ** 2 - dz ** 2, 0.0))
        print(f"{name}: 閉じた形 {ok}  大きさ {(m.bounds[1] - m.bounds[0]).round(1).tolist()}  "
              f"体積 {m.volume / 1000:.2f}cm3（PLA で約 {m.volume / 1000 * 1.24:.0f}g）  "
              f"ずれてもつかめる {catch:.1f}mm  ボールが抜けられる向き ±{90 - cover:.0f}°")
        assert ok, "閉じた形になっていない"


if __name__ == "__main__":
    main()
