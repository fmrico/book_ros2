# A Concise Introduction to Robot Programming with ROS2 - Code Repository

[![rolling](https://github.com/fmrico/book_ros2/actions/workflows/rolling.yaml/badge.svg)](https://github.com/fmrico/book_ros2/actions/workflows/rolling.yaml)
[![jazzy-devel](https://github.com/fmrico/book_ros2/actions/workflows/jazzy-devel.yaml/badge.svg)](https://github.com/fmrico/book_ros2/actions/workflows/jazzy-devel.yaml)
[![GitHub Action
Status](https://github.com/fmrico/book_ros2/actions/workflows/humble-devel.yaml/badge.svg?branch=humble-devel)](https://github.com/fmrico/book_ros2)
[![GitHub Action
Status](https://github.com/fmrico/book_ros2/workflows/foxy-devel/badge.svg)](https://github.com/fmrico/book_ros2)

## Code

This repository contains the source code shown and analyzed in the book _A Concise Introduction to Robot Programming with ROS2_, 2nd Edition, as well as complementary teaching material that will be added.

**Requirements for `rolling` branch**: Ubuntu 24.04 LTS + ROS 2 Rolling Ridley

**Requirements for `jazzy-devel` branch**: Ubuntu 24.04 LTS + ROS 2 Jazzy Jalisco

**Requirements for `humble-devel` branch**: Ubuntu 22.04 LTS + ROS 2 Humble Hawksbill

**Requirements for `foxy-devel` branch**: Ubuntu 20.04 LTS + ROS 2 Foxy Fitzroy

## Slides

* [Slides in PDF](https://www.dropbox.com/s/jgxuyz02wupkie6/BR2_Chapters_PDF.zip?dl=0)
* [Slides in Keynote](https://www.dropbox.com/s/ge56cw4j2v7e6df/BR2_Chapters_KEY.zip?dl=0)
* [Slides in Powerpoint](https://www.dropbox.com/s/s6y5z33ofsm2blw/BR2_Chapters_PPT.zip?dl=0)

![Cover](https://github.com/user-attachments/assets/42b96213-3f52-471f-a8b8-c0a186bee627)

Order book: [https://www.routledge.com/A-Concise-Introduction-to-Robot-Programming-with-ROS2](https://www.routledge.com/A-Concise-Introduction-to-Robot-Programming-with-ROS-2/Rico/p/book/9781032851488?srsltid=AfmBOooKCMPhG5Bsf330d66CeeCnSCSIJaEDh_ShdMKa82awlmNFpnCz)

## RoboStack + Pixi (Jazzy and Kilted)

These steps assume a `colcon` workspace with `src/` and this repo cloned into `src/book_ros2/`. The `build/`, `install/`, and `log/` folders will be created automatically after the first build.

This repository includes a Pixi file at `src/book_ros2/pixi.toml` so you can create a reproducible **RoboStack** environment and build/run the workspace with ROS 2 **Jazzy** or **Kilted**.

### 1) Install Pixi

Follow the official guide: https://pixi.sh/latest/#installation

Linux example:

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

Restart your terminal (or reload your shell) so `pixi` is available in your `PATH`.

### 2) Copy `pixi.toml` to the workspace root

Pixi expects `pixi.toml` at the workspace root directory (the parent directory of `src/`).

From the workspace root:

```bash
cp src/book_ros2/pixi.toml ./pixi.toml
```

### 3) Install dependencies (choose Jazzy or Kilted)

```bash
# ROS 2 Jazzy
pixi install -e jazzy

# ROS 2 Kilted
pixi install -e kilted
```

### 4) Build the workspace with Pixi

In `pixi.toml` there is a `build` task that builds with `colcon` and disables tests to avoid failures due to system dependencies.

```bash
# Compilar con Jazzy
pixi run -e jazzy build

# Compilar con Kilted
pixi run -e kilted build
```

### 5) Open an environment shell (and use ROS 2)

You can open an interactive shell inside the environment. In this project, activation is configured to run `source install/setup.bash` automatically.

```bash
pixi shell -e jazzy
# o
pixi shell -e kilted
```

Inside that shell you should be able to run commands like `ros2 pkg list` or launch nodes from the workspace.

If you prefer to run a command without opening an interactive shell:

```bash
pixi run -e jazzy ros2 run <package> <executable>
pixi run -e jazzy ros2 launch <package> <launch.py>
```

### 6) Run `rmw_zenohd`

The environment sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp` automatically. It is recommended to start the `rmw_zenohd` daemon in a separate terminal.

Option A (recommended): inside `pixi shell -e <distro>`

```bash
rmw_zenohd
```

Option B (without opening an interactive shell):

```bash
pixi run -e jazzy rmw_zenohd
# o
pixi run -e kilted rmw_zenohd
```

To stop it: `Ctrl+C`.