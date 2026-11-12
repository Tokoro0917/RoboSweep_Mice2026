"""
部品の寸法を直すスクリプト（hardware/suction-effector で実行）
    pip install trimesh manifold3d numpy
    python tools/fix_parts.py
    python tools/split_effector.py robosweep_effector_assembled.stl .

1. 組み立てた形: 取り付けアームの下面（z=45.8）がインペラ室の中に 0.6mm はみ出していて、
   インペラ室の高さが 7.4mm（インペラは 7.0mm）しかなかった。
   インペラ室の中（半径 16.5mm 以内、z=38.4〜46.4）を空け、天井の高さを 8.0mm にそろえる。
   元の形 robosweep_effector_original.stl から robosweep_effector_assembled.stl を作る。
2. インペラ: 円板の上に出ているハブを 1mm 短くする（高さ 9.5mm → 8.5mm）。
   軸穴 Φ1.2 は印刷すると細くなり、モーターの軸（Φ1.2）が入らないので、
   軸穴を Φ1.3 / Φ1.4 / Φ1.5 に広げたものも作る。
   元のインペラ（ハブを切る前）は robosweep_impeller_original.stl。
"""
import trimesh
from trimesh.creation import cylinder

CHAMBER_R = 16.5
CHAMBER_BOTTOM = 38.4
CHAMBER_TOP = 46.4

m = trimesh.load("robosweep_effector_original.stl")
cx, cy = (m.bounds[0][:2] + m.bounds[1][:2]) / 2
cut = cylinder(radius=CHAMBER_R - 0.01, height=CHAMBER_TOP - CHAMBER_BOTTOM, sections=128)
cut.apply_translation([cx, cy, (CHAMBER_BOTTOM + CHAMBER_TOP) / 2])
m = trimesh.boolean.difference([m, cut], engine="manifold")
m.export("robosweep_effector_assembled.stl")
print("組み立てた形: 閉じた形", m.is_watertight, "体積 %.2fcm3" % (m.volume / 1000))

IMP_HEIGHT = 8.5   # ハブの上端（元は 9.5）

imp = trimesh.load("robosweep_impeller_original.stl")
keep = trimesh.creation.box(extents=[100, 100, IMP_HEIGHT + 10])
keep.apply_translation([0, 0, (IMP_HEIGHT - 10) / 2])          # z: -10 .. IMP_HEIGHT
imp = trimesh.boolean.intersection([imp, keep], engine="manifold")
imp.export("robosweep_impeller_d1.2.stl")
print("インペラ 軸穴Φ1.2: 閉じた形", imp.is_watertight, "高さ %.2fmm" % imp.bounds[1][2])
for d in (1.3, 1.4, 1.5):
    hole = cylinder(radius=d / 2, height=30, sections=48)
    out = trimesh.boolean.difference([imp, hole], engine="manifold")
    out.export(f"robosweep_impeller_d{d}.stl")
    print(f"インペラ 軸穴Φ{d}: 閉じた形", out.is_watertight, "高さ %.2fmm" % out.bounds[1][2])
