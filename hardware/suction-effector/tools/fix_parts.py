"""
部品の寸法を直すスクリプト（hardware/suction-effector で実行）
    pip install trimesh manifold3d numpy
    python tools/fix_parts.py
    python tools/split_effector.py robosweep_effector_assembled.stl .
    python tools/add_press_fit.py

受け取ったままの元の形（*_original.stl）から作り直すので、何回実行しても結果は同じ。
吸着ユニットの軸は (x, y) = (0, 0)。取り付けアームがあるので、外形の中心とは 0.21mm ずれている。

1. 組み立てた形（robosweep_effector_original.stl → robosweep_effector_assembled.stl）
   - インペラ室の高さ: 8mm → 10mm（天井を 2mm 上げる。z=38.4〜48.4）。
     インペラ（羽根 6mm + 円板 1mm）の上下のすき間は 1.5mm ずつになる。
     元の形では取り付けアームの下面（z=45.8）がインペラ室に 0.6mm はみ出していたので、
     インペラ室の中はまるごと空ける。
   - インペラ室の内径は元のまま Φ33（インペラ Φ30 との横のすき間 1.5mm）。
   - モーターの止め: 天井の上面（z=49.6）でモーター（Φ8.6）が止まる。
     天井の穴は Φ5.6 で、インペラのハブ（Φ5）はこの穴に入る。
2. インペラ（robosweep_impeller_original.stl → robosweep_impeller_d*.stl）
   - 円板の上に出ているハブを 1mm 短くする（高さ 9.5mm → 8.5mm）。
   - 軸穴 Φ1.2 は印刷すると細くなり、モーターの軸（Φ1.2）が入らないので、
     Φ1.3 / Φ1.4 / Φ1.5 に広げたものを作る（P2S では Φ1.5 がちょうどよかった）。
"""
import trimesh
from trimesh.creation import box, cylinder

CX, CY = 0.0, 0.0           # 吸着ユニットの軸
WALL_R = 20.60              # 足す壁の外半径（元の壁 20.62 とわずかにずらして、面が重ならないようにする）
CHAMBER_R = 16.55           # インペラ室の内半径（Φ33.1。元の 16.495 とわずかにずらす）
CHAMBER_BOTTOM = 38.4       # 床（吸込口の板の上面）
CHAMBER_TOP = 48.4          # 天井の下面（元は 46.4）
CEILING_TOP = 49.6          # 天井の上面 = モーターが止まる面（元は 47.6）
OLD_CEILING_TOP = 47.6
STOP_HOLE_R = 2.8           # 天井の穴（Φ5.6）
BORE_R = 4.42               # モーターのボスの穴（Φ8.84。元の 4.40 とわずかにずらす）
BORE_TOP = 59.6             # ボスの上端

IMP_HEIGHT = 8.5            # インペラの高さ（ハブの上端。元は 9.5）


def cyl(r, z0, z1, sections=128):
    c = cylinder(radius=r, height=z1 - z0, sections=sections)
    c.apply_translation([CX, CY, (z0 + z1) / 2])
    return c


# ---- 組み立てた形 ----
import numpy as np
from trimesh.creation import revolve

SECTIONS = 96


def ring(r0, r1, z0, z1):
    """回転体の輪（真ん中から放射状に分割しないので、ほかの円とぶつかっても細い三角形ができにくい）"""
    if r0 <= 0:
        prof = np.array([[0, z0], [r1, z0], [r1, z1], [0, z1]])
    else:
        prof = np.array([[r0, z0], [r1, z0], [r1, z1], [r0, z1], [r0, z0]])
    m = revolve(prof, sections=SECTIONS)
    m.apply_translation([CX, CY, 0])
    return m


m = trimesh.load("robosweep_effector_original.stl")
m = trimesh.boolean.difference([m, ring(0, BORE_R, OLD_CEILING_TOP - 1.5, BORE_TOP + 1)], engine="manifold")  # 元の穴を 4.42 で削り直す
# 壁と天井を 2mm 上へ（天井の穴 Φ5.6 = モーターの止めも、この輪で作る）
m = trimesh.boolean.union([m, ring(STOP_HOLE_R, WALL_R, OLD_CEILING_TOP - 1.5, CEILING_TOP)], engine="manifold")
m = trimesh.boolean.difference([m,
                                ring(0, CHAMBER_R, CHAMBER_BOTTOM, CHAMBER_TOP),     # インペラ室
                                ring(0, BORE_R, CEILING_TOP, BORE_TOP + 1)],         # モーターの穴
                               engine="manifold")
# ほぼ同じ位置の頂点（1e-4mm 以内）をまとめ、つぶれた三角形を消してから保存する
m.merge_vertices(digits_vertex=4)
m.update_faces(m.nondegenerate_faces(height=1e-6))
m.remove_unreferenced_vertices()
assert m.is_volume, "閉じた形になっていない"
m.export("robosweep_effector_assembled.stl")
assert trimesh.load("robosweep_effector_assembled.stl").is_volume, "保存した STL が閉じた形になっていない"
print("組み立てた形: 閉じた形", m.is_volume, "体積 %.2fcm3" % (m.volume / 1000))

# ---- インペラ ----
imp = trimesh.load("robosweep_impeller_original.stl")
keep = box(extents=[100, 100, IMP_HEIGHT + 10])
keep.apply_translation([0, 0, (IMP_HEIGHT - 10) / 2])          # z: -10 .. IMP_HEIGHT
imp = trimesh.boolean.intersection([imp, keep], engine="manifold")
imp.export("robosweep_impeller_d1.2.stl")
print("インペラ 軸穴Φ1.2: 閉じた形", imp.is_volume, "高さ %.2fmm" % imp.bounds[1][2])
for d in (1.3, 1.4, 1.5):
    hole = cylinder(radius=d / 2, height=30, sections=48)
    out = trimesh.boolean.difference([imp, hole], engine="manifold")
    out.export(f"robosweep_impeller_d{d}.stl")
    print(f"インペラ 軸穴Φ{d}: 閉じた形", out.is_volume, "高さ %.2fmm" % out.bounds[1][2])
