# Kobuki + RPLidar A2 Setup

## Setup Kobuki base

1. Connect the Kobuki to your laptop by USB (**Switch On the robot**)
2. Check that the device has `rw` permission for `dialout` group, and you are in this group

```
ls /dev/ttyUSB0
id
```

If you are not in `dialout` group, add yourself to this group:

```
useradd ${USER} dialout
```

Logout and login again.

4. Clone this repo in your workspace. Let's assume that your workspace is `~/ros2_ws`

```
cd ~/ros2_ws/src
gir clone -b foxy https://github.com/IntelligentRoboticsLabs/Robots.git
```

6. Download the repos with the kobuki driver and utils

```
cd ~/ros2_ws/src
vcs import . < Robots/kobuki/third_parties.repos
```

7. Try to install from packages as much dependencies as possible

```
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

8. Compile the workspace

```
cd ~/ros2_ws
colcon build --symlink-install
```

9. Source the workspace or open a new terminal (if sources in .bashrc)
10. Use the kobuki utils to create the udev rules

```
ros2 run kobuki_ftdi create_udev_rules
```

11. Let's launch the kobuki driver to test that everything went ok

On terminal 1:
```
ros2 launch kobuki_node minimal.launch
```

You should have listened a sound that indicates that the driver succesfully communicated with the robot. Check permissions otherwise

On terminal 2:
```
ros2 topic list
```
You should be seeing the topics

## Setup RPLidar A2

The drivers have been already downloaded and built in the previous sections, but we need to manually create the udev rules

1. Create the file `/dev/udev.d/rplidar.xxx` with this content
2. restart udev 

```
sudo udevadm control --reload-rules && sudo udevadm trigger
```

4. Run the driver

```
ros2 launch rplidar rplidar.launch.py
```

5. Visualize the `/scan` topic in RViz2 (frame `laser`)

## Launch everything

```
ros2 run mr2_kobuki kobuki_rplidar.launch.py
```

Open RViz2 and check TFs and Laser

## Launch kobuki simulated

```
ros2 run mr2_kobuki sim.launch.py
```
