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

3. Use the kobuki utils to create the udev rules

```
ros2 run kobuki_ftdi create_udev_rules
```

4. Let's launch the kobuki driver to test that everything went ok

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

1. Create the file `/dev/udev.d/10-rplidar.rules` with this content

```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lidar"
```

2. restart udev 

```
sudo udevadm control --reload-rules && sudo udevadm trigger
```

3. Run the driver

```
ros2 launch rplidar rplidar.launch.py
```

4. Visualize the `/scan` topic in RViz2 (frame `laser`)

## Launch everything

```
ros2 run mr2_kobuki kobuki_rplidar.launch.py
```

Open RViz2 and check TFs and Laser

