# OMPL Path Planning Demo

Self-contained C++ project using OMPL to plan a collision-free SE(3) path for a cylinder inserting into a cube with a cylindrical hole. The project generates PLY meshes, runs multiple planners, and can render a GIF of the motion.

## Prerequisites

- CMake >= 3.16
- C++17 compiler
- Python 3 (for GIF rendering)

## Build

```bash
cd /Users/saiganesh/Documents/path_planning_ompl
cmake -S . -B build
cmake --build build -j 2
```

## Generate Meshes

```bash
./build/generate_ply data
```

This creates:
- `data/cylinder.ply`
- `data/cube_with_hole.ply`

## Run Planners

```bash
./build/plan_path data output
```

Outputs:
- `output/RRTstar_path.csv`
- `output/PRMstar_path.csv`
- `output/BITstar_path.csv`
- `output/*_summary.txt`

## Render GIF

Install Python deps:

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install imageio matplotlib numpy
```

Render a GIF (example: RRT*):

```bash
python3 scripts/render_gif.py output/RRTstar_path.csv output/RRTstar.gif
```

## Notes

- OMPL is fetched and built as part of the CMake configure step.
- Boost is built locally via `scripts/build_boost.sh` to keep the project self-contained.
