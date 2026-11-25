# XCR Continuum Robot

A bio-inspired elephant-trunk continuum robot (XCR arm), built with ROS 2 Humble.
Three stepper motors pull three tendons tied to the last ring, which bends the
whole 3-segment body (a TPU helical lattice) up to about 90°.

## Repository layout

| Folder | Runs on | Packages |
|---|---|---|
| `pc_trunk_ws/` | Ubuntu 22.04 PC | `trunk_bringup`, `trunk_control`, `trunk_description`, `trunk_sim` |
| `rpi_trunk_ws/` | Raspberry Pi 4 (`hegzo`) | `trunk_control` |

Each folder is its own colcon workspace. **Build from inside each workspace,
not from the repo root.** Don't source both workspaces in the same terminal:
both contain a package called `trunk_control`.

## System overview

```
PC 10.42.0.1 ──Ethernet switch── Raspberry Pi 4 10.42.0.239 ──USB── Arduino Due ── 3 steppers
                     └── Vicon tracking
```
Middleware: CycloneDDS on both machines.

## Getting started

See `pc_trunk_ws/README.md` and `rpi_trunk_ws/README.md`.
