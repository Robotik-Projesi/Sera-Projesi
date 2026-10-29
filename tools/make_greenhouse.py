#!/usr/bin/env python3
"""
Sera "olgunluk tarayici" projesi - LiDAR Algilayabilir Kucuk Engel Destekli Dunya Uretici.

Uretilenler:
  - greenhouse.sdf     : Gazebo dunyasi
  - ground_truth.json  : Domateslerin gercek konumlari ve sayilari
"""

import argparse
import json
import math
import os
import random

# ------------------------------------------------------------------ renkler
RED_SHADES = [(0.90, 0.08, 0.08), (0.85, 0.10, 0.06), (0.80, 0.07, 0.07)]
UNRIPE_SHADES = [(0.60, 0.90, 0.15), (0.65, 0.88, 0.20), (0.55, 0.85, 0.18)]
LEAF_SHADES = [(0.08, 0.32, 0.10), (0.12, 0.40, 0.12), (0.10, 0.28, 0.09)]
BACK = (0.06, 0.22, 0.07)
STEM = (0.20, 0.32, 0.10)
CALYX = (0.10, 0.35, 0.10)
WOOD = (0.45, 0.30, 0.18)
SOIL_DARK = (0.22, 0.15, 0.09)
SOIL = (0.50, 0.45, 0.40)
PATH = (0.62, 0.58, 0.52)
RAMP = (0.50, 0.50, 0.50)
OBSTACLE_COLOR = (0.20, 0.20, 0.20)
GLASS = (0.80, 0.90, 1.00)
FRAME = (0.85, 0.85, 0.85)

TOMATO_R = 0.04


# -------------------------------------------------------------- yardimcilar
def rgba(c, a=1.0):
    return f"{c[0]:.3f} {c[1]:.3f} {c[2]:.3f} {a:.1f}"


def material(c):
    return (
        f"<material><ambient>{rgba(c)}</ambient><diffuse>{rgba(c)}</diffuse>"
        f"<specular>0.05 0.05 0.05 1</specular></material>"
    )


def g_box(sx, sy, sz):
    return f"<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>"


def g_cyl(r, l):
    return f"<geometry><cylinder><radius>{r:.4f}</radius><length>{l:.3f}</length></cylinder></geometry>"


def g_sph(r):
    return f"<geometry><sphere><radius>{r:.4f}</radius></sphere></geometry>"


def vis(name, geom, pose, color, transp=None):
    t = f"<transparency>{transp}</transparency>" if transp is not None else ""
    return (
        f"<visual name='{name}'><pose>{pose}</pose>{geom}{material(color)}{t}</visual>"
    )


def pz(x, y, z, r=0.0, p=0.0, yw=0.0):
    return f"{x:.4f} {y:.4f} {z:.4f} {r:.4f} {p:.4f} {yw:.4f}"


# --------------------------------------------------- rampa & engel (collision)
def box_link(name, size, pose, color):
    geom = g_box(*size)
    return (
        f"<link name='{name}'><pose>{pose}</pose>"
        f"<collision name='{name}_col'>{geom}</collision>"
        f"<visual name='{name}_vis'>{geom}{material(color)}</visual></link>"
    )


def ramp_links(x_start, length, width, thickness, angle_deg, prefix, y_pos=0.6):
    th = math.radians(angle_deg)
    c, s = math.cos(th), math.sin(th)
    zc = (length / 2) * s - (thickness / 2) * c
    x_up = x_start + (length / 2) * c + (thickness / 2) * s
    x_peak = x_start + length * c
    x_down = x_peak + (length / 2) * c - (thickness / 2) * s
    size = (length, width, thickness)
    up = box_link(f"{prefix}_up", size, pz(x_up, y_pos, zc, 0, -th, 0), RAMP)
    down = box_link(f"{prefix}_down", size, pz(x_down, y_pos, zc, 0, th, 0), RAMP)
    return up + down, length * s


def lidar_obstacle_link(name, x, y_pos):
    """LiDAR tarafindan rahatca algilanan kucuk kutu engel (25x25x12 cm)."""
    sx, sy, sz = 0.25, 0.25, 0.12
    # Yolun bir kenarinda durur, koridoru tamamen kapatmaz
    pose = pz(x, y_pos - 0.15, sz / 2.0)
    geom = g_box(sx, sy, sz)
    return (
        f"<link name='{name}'><pose>{pose}</pose>"
        f"<collision name='{name}_col'>{geom}</collision>"
        f"<visual name='{name}_vis'>{geom}{material(OBSTACLE_COLOR)}</visual></link>"
    )


# ------------------------------------------------------------- domatesler
def place_tomatoes(args, rng):
    slots = []
    n = int((args.row_length - 2 * args.margin) / args.slot_step) + 1
    
    y_faces = [
        ("left", 1.2 - args.row_width / 2),
        ("center_left", 0.0 + args.row_width / 2),
        ("center_right", 0.0 - args.row_width / 2),
        ("right", -1.2 + args.row_width / 2)
    ]

    for face_name, y_pos in y_faces:
        for i in range(n):
            slots.append((face_name, y_pos, args.margin + i * args.slot_step))

    total = args.ripe + args.unripe
    if total > len(slots):
        raise SystemExit(
            f"Hata: {total} domates istendi ama sadece {len(slots)} slot var."
        )

    chosen = rng.sample(slots, total)
    kinds = ["ripe"] * args.ripe + ["unripe"] * args.unripe
    rng.shuffle(kinds)

    tomatoes = []
    for idx, ((face_name, y_base, x0), kind) in enumerate(zip(chosen, kinds)):
        tomatoes.append(
            {
                "id": idx,
                "kind": kind,
                "face": face_name,
                "x": round(x0 + rng.uniform(-0.08, 0.08), 3),
                "y": round(y_base + rng.uniform(-0.02, 0.02), 3),
                "z": round(rng.uniform(args.z_min, args.z_max), 3),
            }
        )
    tomatoes.sort(key=lambda t: t["x"])
    return tomatoes


def build_tomatoes(tomatoes, drng):
    links = ""
    for t in tomatoes:
        shades = RED_SHADES if t["kind"] == "ripe" else UNRIPE_SHADES
        v = vis("fruit", g_sph(TOMATO_R), pz(0, 0, 0), drng.choice(shades))
        v += vis("calyx", g_sph(0.012), pz(0, 0, TOMATO_R - 0.006), CALYX)
        v += vis(
            "pedicel",
            g_cyl(0.003, 0.07),
            pz(0, 0.045, TOMATO_R - 0.005, math.pi / 2, 0, 0),
            STEM,
        )
        links += (
            f"<link name='tomato_{t['kind']}_{t['id']:02d}'>"
            f"<pose>{pz(t['x'], t['y'], t['z'])}</pose>{v}</link>"
        )
    return f"<model name='tomatoes'><static>true</static>{links}</model>"


# ------------------------------------------------------------ bitki sirasi
def build_rows(args, drng):
    rw, L, H = args.row_width, args.row_length, args.row_height
    rows = [("row_left", 1.2), ("row_center", 0.0), ("row_right", -1.2)]
    out = ""

    for name, y_pos in rows:
        v = vis(f"{name}_trough", g_box(L, rw, 0.20), pz(L / 2, y_pos, 0.10), WOOD)
        v += vis(f"{name}_soil", g_box(L - 0.1, rw - 0.08, 0.02), pz(L / 2, y_pos, 0.21), SOIL_DARK)
        
        n_plants = int((L - 0.4) / args.plant_step) + 1
        for i in range(n_plants):
            px = 0.2 + i * args.plant_step
            v += vis(
                f"{name}_stem{i}",
                g_cyl(0.012, H - 0.2),
                pz(px, y_pos, 0.2 + (H - 0.2) / 2),
                STEM,
            )
            for k in range(args.leaves):
                z = drng.uniform(0.22, H - 0.02)
                d = drng.uniform(-rw / 2.2, rw / 2.2)
                v += vis(
                    f"{name}_leaf{i}_{k}",
                    g_box(drng.uniform(0.08, 0.13), drng.uniform(0.04, 0.07), 0.004),
                    pz(
                        px + drng.uniform(-0.18, 0.18),
                        y_pos + d,
                        z,
                        drng.uniform(-0.6, 0.6),
                        drng.uniform(-0.8, 0.8),
                        drng.uniform(-math.pi, math.pi),
                    ),
                    drng.choice(LEAF_SHADES),
                )
            v += vis(
                f"{name}_clump{i}",
                g_sph(drng.uniform(0.07, 0.10)),
                pz(px + drng.uniform(-0.05, 0.05), y_pos, H),
                drng.choice(LEAF_SHADES),
            )
        c = (
            f"<collision name='{name}_col'><pose>{pz(L / 2, y_pos, H / 2)}</pose>"
            f"{g_box(L, rw, H)}</collision>"
        )
        out += f"<link name='{name}'>{c}{v}</link>"
    return f"<model name='plant_rows'><static>true</static>{out}</model>"


# ------------------------------------------------------------ sera yapisi
def build_structure(args):
    L = args.row_length
    W = 1.8
    H = 2.0
    a = math.radians(20)
    xc, Ltot = L / 2, L + 4
    v = ""
    for s, tag in ((+1, "l"), (-1, "r")):
        v += vis(f"wall_{tag}", g_box(Ltot, 0.01, H), pz(xc, s * W, H / 2), GLASS, 0.85)
        panel_w = W / math.cos(a)
        v += vis(
            f"roof_{tag}",
            g_box(Ltot, panel_w, 0.01),
            pz(xc, s * W / 2, H + (W / 2) * math.tan(a), -s * a, 0, 0),
            GLASS,
            0.80,
        )
        k = 0
        x = -2.0
        while x <= L + 2.0 + 1e-6:
            v += vis(f"post_{tag}{k}", g_cyl(0.03, H), pz(x, s * W, H / 2), FRAME)
            k += 1
            x += 3.0
    v += vis(
        "ridge",
        g_cyl(0.03, Ltot),
        pz(xc, 0, H + W * math.tan(a), 0, math.pi / 2, 0),
        FRAME,
    )
    return f"<model name='structure'><static>true</static><link name='frame'>{v}</link></model>"


# ------------------------------------------------------------------- dunya
def build_sdf(args, tomatoes, drng):
    cw, L = args.corridor_width, args.row_length

    path_left = vis("v_left", g_box(L + 4, cw, 0.004), pz(L / 2, 0.6, 0.002), PATH)
    path_right = vis("v_right", g_box(L + 4, cw, 0.004), pz(L / 2, -0.6, 0.002), PATH)
    path_model = f"<model name='path'><static>true</static><link name='l'>{path_left}{path_right}</link></model>"

    ramp_xml, ramp_h = ramp_links(
        args.ramp_x, args.ramp_length, args.ramp_width, 0.05, args.ramp_angle, "ramp", y_pos=0.6
    )
    structure = "" if args.no_structure else build_structure(args)

    sdf = f"""<?xml version="1.0" ?>
<!-- OTOMATIK URETILDI: make_greenhouse.py (seed={args.seed}). -->
<sdf version="1.10">
  <world name="greenhouse">
    <physics name="default_physics" type="ignored">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.75 0.85 0.95 1</background>
      <shadows>false</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.3 0.2 -1</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="ground_link">
        <collision name="ground_col">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="ground_vis">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          {material(SOIL)}
        </visual>
      </link>
    </model>

    {path_model}
    {build_rows(args, drng)}
    {build_tomatoes(tomatoes, drng)}
    <model name='bump'><static>true</static>{ramp_xml}</model>
    <model name='corridor_obstacle'><static>true</static>{lidar_obstacle_link('obstacle_box', args.obstacle_x, y_pos=-0.6)}</model>
    {structure}
  </world>
</sdf>
"""
    return sdf, ramp_h


def main():
    p = argparse.ArgumentParser(description="2 Koridorlu Sera Dunyasi Uretici")
    p.add_argument("--out", default="greenhouse.sdf", help="cikti SDF yolu")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--ripe", type=int, default=24, help="olgun domates sayisi")
    p.add_argument("--unripe", type=int, default=18, help="ham domates sayisi")
    p.add_argument("--row_length", type=float, default=9.0)
    p.add_argument("--row_width", type=float, default=0.4)
    p.add_argument("--row_height", type=float, default=0.8)
    p.add_argument("--corridor_width", type=float, default=0.8)
    p.add_argument("--slot_step", type=float, default=0.25)
    p.add_argument("--margin", type=float, default=0.8)
    p.add_argument("--z_min", type=float, default=0.25)
    p.add_argument("--z_max", type=float, default=0.60)
    p.add_argument("--plant_step", type=float, default=0.40)
    p.add_argument("--leaves", type=int, default=9)
    p.add_argument("--ramp_x", type=float, default=3.0)
    p.add_argument("--ramp_length", type=float, default=1.2)
    p.add_argument("--ramp_width", type=float, default=0.7)
    p.add_argument("--ramp_angle", type=float, default=6.0)
    p.add_argument("--obstacle_x", type=float, default=6.5, help="Engel X konumu")
    p.add_argument("--no_structure", action="store_true")
    args = p.parse_args()

    rng = random.Random(args.seed)
    drng = random.Random(args.seed + 1000)
    tomatoes = place_tomatoes(args, rng)
    sdf, ramp_h = build_sdf(args, tomatoes, drng)

    out_dir = os.path.dirname(os.path.abspath(args.out))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(sdf)

    gt = {
        "seed": args.seed,
        "corridors": 2,
        "ripe_total": sum(t["kind"] == "ripe" for t in tomatoes),
        "unripe_total": sum(t["kind"] == "unripe" for t in tomatoes),
        "tomatoes": tomatoes,
    }
    gt_path = os.path.join(out_dir, "ground_truth.json") if out_dir else "ground_truth.json"
    with open(gt_path, "w", encoding="utf-8") as f:
        json.dump(gt, f, indent=2)

    print(f"SDF yazildi         : {args.out}")
    print(f"Ground truth yazildi: {gt_path}")


if __name__ == "__main__":
    main()
