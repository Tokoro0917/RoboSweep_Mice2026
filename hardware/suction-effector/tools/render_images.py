"""
説明用の画像を作るスクリプト（hardware/suction-effector で実行）
    pip install trimesh manifold3d numpy matplotlib
    python tools/render_images.py
"""
import numpy as np
import trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.collections import LineCollection
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from trimesh.intersections import mesh_plane

for f in ("IPAGothic", "IPAexGothic", "Noto Sans CJK JP"):
    if any(f in x.name for x in font_manager.fontManager.ttflist):
        plt.rcParams["font.family"] = f
        break

Z_CUT = 38.4
SKIRT_H = 4.5        # はめ込みのスカートの高さ
IMP_Z = 38.9          # インペラの羽根の下端（床から 0.5mm）
MOTOR_D = 8.6
MOTOR_BOTTOM = 48.9   # モーターの下面（ハブの上から 0.5mm）


def load_assembled():
    lower = trimesh.load("robosweep_effector_lower.stl")
    upper = trimesh.load("robosweep_effector_upper_fit0.1.stl")
    imp = trimesh.load("robosweep_impeller_d1.5.stl")
    # 組み立てた向きに戻す（上の部品はスカートの下端が z=0）
    lower.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0]))
    lower.apply_translation([0, 0, -lower.bounds[0][2]])
    upper.apply_translation([0, 0, Z_CUT - SKIRT_H])
    imp.apply_translation([0, 0, IMP_Z])
    return lower, upper, imp


def section_lines(mesh, normal):
    seg = mesh_plane(mesh, plane_normal=normal, plane_origin=[0, 0, 0])
    ax = 0 if normal[1] else 1
    return [[(s[0, ax], s[0, 2]), (s[1, ax], s[1, 2])] for s in seg]


def fig_section():
    lower, upper, imp = load_assembled()
    fig, axes = plt.subplots(1, 2, figsize=(14, 8.2), gridspec_kw={"width_ratios": [1.25, 1]})
    for ax, (xlim, ylim, title) in zip(axes, [((-50, 50), (-3, 72), "断面図（組み立てた状態・ボールを吸う向き）"),
                                              ((-22, 22), (35, 62), "インペラ室の拡大")]):
        for mesh, color, lw in ((lower, "#555555", 1.4), (upper, "#b5651d", 1.4), (imp, "#1f77b4", 1.6)):
            ax.add_collection(LineCollection(section_lines(mesh, [0, 1, 0]), colors=color, linewidths=lw))
        ax.add_patch(Rectangle((-MOTOR_D / 2, MOTOR_BOTTOM), MOTOR_D, 20, fill=False, ec="#d62728", lw=1.6, ls="--"))
        ax.plot([0, 0], [MOTOR_BOTTOM - 6, MOTOR_BOTTOM], color="#d62728", lw=2.2)
        ax.axhline(Z_CUT, color="#2ca02c", lw=1, ls=":")
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_aspect("equal")
        ax.set_title(title, fontsize=13)
        ax.set_xlabel("mm")
        ax.grid(alpha=0.2)

    a = axes[0]
    a.text(-17, 6, "コーン（口径 Φ90）\nこの口を下にして\nボールにかぶせる", fontsize=10, color="#555555")
    a.annotate("吸着座 Φ35\n（縁にスポンジ）", xy=(-17.5, 32), xytext=(-48, 24), fontsize=10,
               arrowprops=dict(arrowstyle="->", color="#555555"))
    a.annotate("吸込口 Φ18", xy=(-9, 37.8), xytext=(-48, 37), fontsize=10,
               arrowprops=dict(arrowstyle="->", color="#555555"))
    a.annotate("インペラ（青）\n羽根を下に向ける", xy=(-12, 41), xytext=(-48, 46), fontsize=10, color="#1f77b4",
               arrowprops=dict(arrowstyle="->", color="#1f77b4"))
    a.annotate("モーター（赤の点線）\n上から差し込む・軸は下向き", xy=(4.3, 60), xytext=(10, 66), fontsize=10, color="#d62728",
               arrowprops=dict(arrowstyle="->", color="#d62728"))
    a.annotate("取り付けアーム（M3 × 2）", xy=(35, 48.8), xytext=(18, 55), fontsize=10, color="#b5651d",
               arrowprops=dict(arrowstyle="->", color="#b5651d"))
    a.text(22, 36.2, "ここで上下に分かれる（緑の点線）", fontsize=9.5, color="#2ca02c")
    a.annotate("はめ込みのスカート\n（下の部品の筒にかぶせる）", xy=(-22.5, 36), xytext=(-48, 51), fontsize=10, color="#b5651d",
               arrowprops=dict(arrowstyle="->", color="#b5651d"))
    a.text(-48, -2, "灰色 = 下の部品　茶色 = 上の部品　青 = インペラ　赤 = モーター（長さは実物に合わせる）", fontsize=9)

    b = axes[1]
    b.annotate("天井（床から 8.0mm）", xy=(-14, 46.4), xytext=(-21, 52), fontsize=10,
               arrowprops=dict(arrowstyle="->"))
    b.annotate("排気スリット（6 本）", xy=(18.5, 42.5), xytext=(4, 36), fontsize=10,
               arrowprops=dict(arrowstyle="->"))
    b.annotate("円板とハブ（長いほう）が上", xy=(2.6, 47.5), xytext=(5, 57), fontsize=10, color="#1f77b4",
               arrowprops=dict(arrowstyle="->", color="#1f77b4"))
    b.annotate("軸（Φ1.2）をハブの穴に圧入", xy=(0, 46), xytext=(-21, 59), fontsize=10, color="#d62728",
               arrowprops=dict(arrowstyle="->", color="#d62728"))
    b.text(-21, 35.6, "すき間：羽根の下 0.5mm ／ 円板の上 0.5mm", fontsize=10)
    fig.tight_layout()
    fig.savefig("images/section.png", dpi=130)
    plt.close(fig)


def draw_mesh(ax, mesh, color, elev=25, azim=-60):
    tris = mesh.triangles
    n = mesh.face_normals
    light = np.array([0.4, -0.5, 0.75]); light /= np.linalg.norm(light)
    shade = 0.45 + 0.55 * np.clip(n @ light, 0, 1)
    base = np.array(matplotlib.colors.to_rgb(color))
    fc = np.clip(base[None, :] * shade[:, None], 0, 1)
    ax.add_collection3d(Poly3DCollection(tris, facecolors=fc, edgecolors="none"))
    b = mesh.bounds
    c = b.mean(0); r = (b[1] - b[0]).max() / 2
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()


def fig_parts():
    lower = trimesh.load("robosweep_effector_lower.stl")
    upper = trimesh.load("robosweep_effector_upper_fit0.1.stl")
    imp = trimesh.load("robosweep_impeller_d1.5.stl")
    fig = plt.figure(figsize=(15, 5.6))
    items = [
        (lower, "#9a9a9a", 30, "下の部品（印刷の向き）\nコーンの口が上・サポートなし"),
        (upper, "#d28c4b", 30, "上の部品・はめ込み（印刷の向き）\nスカートが下\nアームの下とスカート内側の輪の下にサポート"),
        (imp, "#4a90d9", 35, "インペラ（印刷の向き）\n羽根が下・円板とハブが上"),
    ]
    for i, (mesh, color, elev, title) in enumerate(items):
        ax = fig.add_subplot(1, 3, i + 1, projection="3d")
        draw_mesh(ax, mesh, color, elev=elev)
        ax.set_title(title, fontsize=12, pad=14)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig("images/parts.png", dpi=120)
    plt.close(fig)


def fig_impeller_bottom():
    imp = trimesh.load("robosweep_impeller_d1.5.stl")
    fig = plt.figure(figsize=(11, 5.4))
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    draw_mesh(ax, imp, "#4a90d9", elev=-35, azim=-60)
    ax.set_title("インペラを下から見た形\n（羽根の側。吸込口 Φ18 に向ける）", fontsize=12)
    ax = fig.add_subplot(1, 2, 2, projection="3d")
    draw_mesh(ax, imp, "#4a90d9", elev=40, azim=-60)
    ax.set_title("インペラを上から見た形\n（円板とハブの側。モーターに向ける）", fontsize=12)
    fig.tight_layout()
    fig.savefig("images/impeller.png", dpi=120)
    plt.close(fig)


if __name__ == "__main__":
    fig_section()
    fig_parts()
    fig_impeller_bottom()
    print("images/section.png, images/parts.png, images/impeller.png")
