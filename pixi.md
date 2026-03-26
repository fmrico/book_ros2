# RoboStack + Pixi (Jazzy and Kilted)

This guide explains how to use **RoboStack** and **Pixi** to build and run this repository in a `colcon` workspace with ROS 2 **Jazzy** or **Kilted**.

These steps assume a `colcon` workspace with `src/` and this repo cloned into `src/book_ros2/`. The `build/`, `install/`, and `log/` folders will be created automatically after the first build.

This repository includes a Pixi file at `src/book_ros2/pixi.toml` so you can create a reproducible environment.

## 1) Install Pixi

Follow the official guide: https://pixi.sh/latest/#installation

Linux example:

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

Restart your terminal (or reload your shell) so `pixi` is available in your `PATH`.

## 2) Copy `pixi.toml` to the workspace root

Pixi expects `pixi.toml` at the workspace root directory (the parent directory of `src/`).

From the workspace root:

```bash
cp src/book_ros2/pixi.toml ./pixi.toml
```

## 3) Install dependencies (choose Jazzy or Kilted)

```bash
# ROS 2 Jazzy
pixi install -e jazzy

# ROS 2 Kilted
pixi install -e kilted
```

## 4) Build the workspace with Pixi

In `pixi.toml` there is a `build` task that builds with `colcon` and disables tests to avoid failures due to system dependencies.

```bash
# Build with Jazzy
pixi run -e jazzy build

# Build with Kilted
pixi run -e kilted build
```

## 5) Open an environment shell (and use ROS 2)

You can open an interactive shell inside the environment. In this project, activation is configured to run `source install/setup.bash` automatically.

```bash
pixi shell -e jazzy
# or
pixi shell -e kilted
```

Inside that shell you should be able to run commands like `ros2 pkg list` or launch nodes from the workspace.

If you prefer to run a command without opening an interactive shell:

```bash
pixi run -e jazzy ros2 run <package> <executable>
pixi run -e jazzy ros2 launch <package> <launch.py>
```

## 6) Run `rmw_zenohd`

The environment sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp` automatically. It is recommended to start the `rmw_zenohd` daemon in a separate terminal.

Option A (recommended): inside `pixi shell -e <distro>`

```bash
rmw_zenohd
```

Option B (without opening an interactive shell):

```bash
pixi run -e jazzy rmw_zenohd
# or
pixi run -e kilted rmw_zenohd
```

To stop it: `Ctrl+C`.
