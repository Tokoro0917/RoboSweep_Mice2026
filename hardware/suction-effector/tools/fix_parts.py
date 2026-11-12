"""
部品の寸法を直すスクリプト（hardware/suction-effector で実行）
    pip install trimesh manifold3d numpy
    python tools/fix_parts.py
    python tools/split_effector.py robosweep_effector_assembled.stl .

1. 組み立てた形: 取り付けアームの下面（z=45.8）がインペラ室の中に 0.6mm はみ出していて、
   インペラ室の高さが 7.4mm（インペラは 7.0mm）しかなかった。
   インペラ室の中（半径 16.5mm 以内、z=38.4〜46.4）を空け、天井の高さを 8.0mm にそろえる。
   何回実行しても結果は同じ。
2. インペラ: 軸穴 Φ1.2 は印刷すると細くなり、モーターの軸（Φ1.2）が入らない。
   軸穴を Φ1.3 / Φ1.4 / Φ1.5 に広げたものを作る。
"""
import trimesh
from trimesh.creation import cylinder

CHAMBER_R = 16.5
CHAMBER_BOTTOM = 38.4
CHAMBER_TOP = 46.4

m = trimesh.load("robosweep_effector_assembled.stl")
cx, cy = (m.bounds[0][:2] + m.bounds[1][:2]) / 2
cut = cylinder(radius=CHAMBER_R - 0.01, height=CHAMBER_TOP - CHAMBER_BOTTOM, sections=128)
cut.apply_translation([cx, cy, (CHAMBER_BOTTOM + CHAMBER_TOP) / 2])
m = trimesh.boolean.difference([m, cut], engine="manifold")
m.export("robosweep_effector_assembled.stl")
print("組み立てた形: 閉じた形", m.is_watertight, "体積 %.2fcm3" % (m.volume / 1000))

imp = trimesh.load("robosweep_impeller_d1.2.stl")
for d in (1.3, 1.4, 1.5):
    hole = cylinder(radius=d / 2, height=30, sections=48)
    out = trimesh.boolean.difference([imp, hole], engine="manifold")
    out.export(f"robosweep_impeller_d{d}.stl")
    print(f"インペラ 軸穴Φ{d}: 閉じた形", out.is_watertight)
