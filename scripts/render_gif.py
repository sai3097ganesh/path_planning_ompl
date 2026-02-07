import csv
import math
import os
from pathlib import Path

import imageio
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import numpy as np


def read_path_csv(path):
    rows = []
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append({k: float(v) for k, v in r.items()})
    return rows


def quat_to_rot(qw, qx, qy, qz):
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
    ])


def cylinder_points(radius, half_height, segments=32, height_segments=8):
    theta = np.linspace(0, 2 * math.pi, segments)
    z = np.linspace(-half_height, half_height, height_segments)
    theta_grid, z_grid = np.meshgrid(theta, z)
    x = radius * np.cos(theta_grid)
    y = radius * np.sin(theta_grid)
    return x, y, z_grid


def transform_points(x, y, z, R, t):
    pts = np.stack([x, y, z], axis=-1)
    pts = pts @ R.T + t
    return pts[..., 0], pts[..., 1], pts[..., 2]


def draw_cube(ax, half):
    r = [-half, half]
    for s in r:
        ax.plot([s, s], [-half, -half], [-half, half], color="black", linewidth=1)
        ax.plot([s, s], [half, half], [-half, half], color="black", linewidth=1)
        ax.plot([-half, half], [s, s], [-half, -half], color="black", linewidth=1)
        ax.plot([-half, half], [s, s], [half, half], color="black", linewidth=1)
        ax.plot([-half, -half], [s, s], [-half, half], color="black", linewidth=1)
        ax.plot([half, half], [s, s], [-half, half], color="black", linewidth=1)


def draw_hole(ax, radius, half, segments=48):
    theta = np.linspace(0, 2 * math.pi, segments)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    ax.plot(x, y, -half * np.ones_like(theta), color="gray", linewidth=1)
    ax.plot(x, y, half * np.ones_like(theta), color="gray", linewidth=1)


def render(path_csv, out_gif, cube_half=0.5, hole_radius=0.2, cyl_radius=0.2, cyl_half_height=0.4):
    rows = read_path_csv(path_csv)
    frames = []

    x0, y0, z0 = cylinder_points(cyl_radius, cyl_half_height)

    for i, r in enumerate(rows):
        fig = plt.figure(figsize=(6, 6))
        ax = fig.add_subplot(111, projection="3d")

        R = quat_to_rot(r["qw"], r["qx"], r["qy"], r["qz"])
        t = np.array([r["x"], r["y"], r["z"]])

        x, y, z = transform_points(x0, y0, z0, R, t)
        ax.plot_surface(x, y, z, color="#4C72B0", alpha=0.6, linewidth=0)

        draw_cube(ax, cube_half)
        draw_hole(ax, hole_radius, cube_half)

        ax.set_xlim(-1.5, 1.5)
        ax.set_ylim(-1.5, 1.5)
        ax.set_zlim(-1.5, 1.5)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.view_init(elev=20, azim=45)
        ax.set_title(f"Step {i + 1}/{len(rows)}")

        fig.canvas.draw()
        # Use a backend-compatible framebuffer
        image = np.asarray(fig.canvas.buffer_rgba())
        image = image[..., :3]
        frames.append(image)
        plt.close(fig)

    imageio.mimsave(out_gif, frames, duration=0.08)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("path_csv")
    parser.add_argument("out_gif")
    parser.add_argument("--cube-half", type=float, default=0.5)
    parser.add_argument("--hole-radius", type=float, default=0.2)
    parser.add_argument("--cyl-radius", type=float, default=0.2)
    parser.add_argument("--cyl-half-height", type=float, default=0.4)
    args = parser.parse_args()

    render(args.path_csv, args.out_gif, args.cube_half, args.hole_radius, args.cyl_radius, args.cyl_half_height)
