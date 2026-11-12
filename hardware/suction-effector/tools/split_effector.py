"""
吸着ユニット（1 つの部品）を、印刷できる 2 つの部品に分けるスクリプト。

  インペラ室が閉じていて、1 つの部品のままではインペラを入れられないため、
  吸込口 Φ18 の板の上面（z = 38.4mm）で上下に分ける。

使い方（hardware/suction-effector で実行）
    pip install trimesh manifold3d numpy
    python tools/split_effector.py robosweep_effector_assembled.stl .

できるもの（どちらも印刷する向きにしてある）
    robosweep_effector_lower.stl : コーン + 吸着座 + 吸込口の板。コーンの口が上
    robosweep_effector_upper.stl : インペラ室 + 天井 + モーターのボス + 取り付けアーム。インペラ室の開いた側が下
"""
import sys

import numpy as np
import trimesh
from trimesh.creation import box

Z_CUT = 38.4   # 吸込口の板の上面 = インペラ室の床


def main(src, outdir):
    m = trimesh.load(src)
    cx, cy = (m.bounds[0][:2] + m.bounds[1][:2]) / 2

    big = 200
    below = box(extents=[big, big, Z_CUT + 10])
    below.apply_translation([cx, cy, (Z_CUT - 10) / 2])      # z: -10 .. Z_CUT
    above = box(extents=[big, big, 100])
    above.apply_translation([cx, cy, Z_CUT + 50])             # z: Z_CUT .. Z_CUT+100

    lower = trimesh.boolean.intersection([m, below], engine="manifold")
    upper = trimesh.boolean.intersection([m, above], engine="manifold")

    # 下の部品: 上下を反転して、切った面を印刷台に置く（コーンの口が上）
    lower.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0], [cx, cy, 0]))
    lower.apply_translation([-cx, -cy, -lower.bounds[0][2]])
    # 上の部品: そのまま（切った面 = インペラ室の開いた側が下）
    upper.apply_translation([-cx, -cy, -upper.bounds[0][2]])

    for name, part in (("robosweep_effector_lower.stl", lower),
                       ("robosweep_effector_upper.stl", upper)):
        part.export(f"{outdir}/{name}")
        size = (part.bounds[1] - part.bounds[0]).round(2).tolist()
        print(f"{name}: 閉じた形={part.is_watertight} 体積={part.volume / 1000:.2f}cm3 大きさ={size}")
    print(f"元の体積={m.volume / 1000:.2f}cm3")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
