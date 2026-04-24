# Legacy: Teensy Spine board + PC demos

This folder is **not** part of the main RobStride Serial/CAN workflow at the repository root. It preserves the original **Ethernet–UDP to CAN** bridge, CMake host applications, and Arduino examples **E01–E06**.

## Layout

| Path | Role |
|------|------|
| `teensy/teensy.ino` | Teensy firmware: UDP ↔ CAN (Robstride O2 style bridge). |
| `apps/` | PC executables (`main.cpp`, demos); build with CMake. |
| `spine_board.cpp`, `spine_board.h`, `utils.h` | Host UDP client and helpers. |
| `CMakeLists.txt` | Builds `apps/` against `spine_board` (expects `robstride_o2/` if present in your tree). |
| `ETHERNET_SETUP.md` | PC ↔ Teensy network notes. |
| `examples_teensy/` | Sketches E01–E06 (see `examples_teensy/README.md` inside this folder). |
| `data/` | Frequency logs, plot script (`plot_frequency.py`), and sample CSV/PNG outputs. |

## Build (from repo root)

```bash
cd legacy
mkdir -p build && cd build
cmake ..
make -j4
```

If CMake reports missing **`robstride_o2/`**, restore or vendor that library from your older checkout; it was referenced by the original top-level `CMakeLists.txt`.

## RobStride Serial controller

The supported sketch for **direct Serial + CAN** control lives at the repo root: **`../Robstride_Controller/`** (not under `legacy/`).
