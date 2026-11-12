"""
上の部品（インペラ室側）に、下の部品へはめ込むスカートを付けるスクリプト
（hardware/suction-effector で実行。先に fix_parts.py と split_effector.py を実行しておく）
    pip install trimesh manifold3d numpy
    python tools/add_press_fit.py

下の部品の吸込口の板の下には、外径 Φ41.67 のまっすぐな筒が 6.4mm ある。
上の部品の下にこの筒の外側にかぶさるスカートを付け、はめ込み（圧入）で止める。
上下方向は、インペラ室の壁の下端が板の上面に当たって止まる（インペラ室の高さ 8mm は変わらない）。

できるもの
    robosweep_effector_upper_fit0.1.stl : スカートの内径 = 筒の外径 + 0.1mm（直径で。きつめ）
    robosweep_effector_upper_fit0.2.stl : + 0.2mm（ふつう）
    robosweep_effector_upper_fit0.3.stl : + 0.3mm（ゆるめ）
    印刷すると穴は細く出るので、どれが合うかは試験リングで確かめる。
    fit_test_skirt.stl : 上の 3 通りのスカートだけの試験リング（切り欠きの数 = 1, 2, 3）
"""
import numpy as np
import trimesh
from trimesh.creation import annulus, box, revolve

PIPE_R = 41.67 / 2       # 下の部品の筒の外半径
SKIRT_H = 4.5            # スカートの高さ（筒のまっすぐな部分は 6.4mm）
SKIRT_T = 1.6            # スカートの厚さ
LEAD = 0.6               # 入口の面取り
JOIN_H = 0.8             # スカートと壁をつなぐ輪の高さ（スリットは床から 1.0mm 上から）
WALL_INNER_R = 16.5
CLEARANCES = (0.1, 0.2, 0.3)


def skirt(clearance, with_join=True):
    """z = -SKIRT_H .. 0 のスカート（z=0 が上の部品の下面 = 板の上面）。clearance は直径の差"""
    r_in = PIPE_R + clearance / 2
    r_out = r_in + SKIRT_T
    # 断面（r, z）を回して作る。入口の内側は面取り
    profile = np.array([
        [r_in + LEAD, -SKIRT_H],
        [r_out, -SKIRT_H],
        [r_out, 0.0],
        [r_in, 0.0],
        [r_in, -SKIRT_H + LEAD],
    ])
    s = revolve(np.vstack([profile, profile[:1]])[:, [0, 1]], sections=256)
    if with_join:
        join = annulus(r_min=WALL_INNER_R + 2.0, r_max=r_out, height=JOIN_H, sections=256)
        join.apply_translation([0, 0, JOIN_H / 2])
        s = trimesh.boolean.union([s, join], engine="manifold")
    return s


def main():
    upper = trimesh.load("robosweep_effector_upper.stl")   # 下面が z=0、中心が原点
    for c in CLEARANCES:
        part = trimesh.boolean.union([upper, skirt(c)], engine="manifold")
        part.apply_translation([0, 0, -part.bounds[0][2]])
        name = f"robosweep_effector_upper_fit{c:.1f}.stl"
        part.export(name)
        print(f"{name}: 閉じた形={part.is_volume} 大きさ={(part.bounds[1] - part.bounds[0]).round(2).tolist()}"
              f" スカート内径=Φ{2 * PIPE_R + c:.2f}")

    # 試験リング: スカートだけ（つなぎの輪の代わりに上に 1mm のつばを付けて、板の上面に当たるようにする）
    rings = []
    for i, c in enumerate(CLEARANCES):
        r = skirt(c, with_join=False)
        r.apply_translation([0, 0, SKIRT_H])                   # z: 0 .. SKIRT_H
        lid = annulus(r_min=PIPE_R - 2.0, r_max=PIPE_R + c / 2 + SKIRT_T, height=1.0, sections=256)
        lid.apply_translation([0, 0, SKIRT_H + 0.5])
        ring = trimesh.boolean.union([r, lid], engine="manifold")
        # 切り欠き（1, 2, 3 個）で見分ける
        for k in range(i + 1):
            notch = box(extents=[2.0, 4.0, 1.2])
            ang = np.radians(20 * k)
            rr = PIPE_R + c / 2 + SKIRT_T
            notch.apply_translation([rr * np.cos(ang), rr * np.sin(ang), SKIRT_H + 0.5])
            ring = trimesh.boolean.difference([ring, notch], engine="manifold")
        # 印刷の向き: つばを下に
        ring.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0]))
        ring.apply_translation([i * 50.0, 0, -ring.bounds[0][2]])
        rings.append(ring)
    test = trimesh.util.concatenate(rings)
    test.export("fit_test_skirt.stl")
    print("fit_test_skirt.stl: 大きさ", (test.bounds[1] - test.bounds[0]).round(2).tolist())


if __name__ == "__main__":
    main()
