# tiago sumulated Setup

1. Download the repos with the tiago simulated

```
cd ~/br2_ws/src
vcs import . < master_ros2/br2_tiago/third_parties.rosinstall
```

2. Try to install from packages as much dependencies as possible

```
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Compile the workspace

```
cd ~/ros2_ws
colcon build --symlink-install
```

4. Source the workspace or open a new terminal (if sources in .bashrc)

5. Let's launch the tiago simulated

On terminal 1:
```
ros2 launch br2_tiago sim.launch.launch
```

You should have listened a sound that indicates that the driver succesfully communicated with the robot. Check permissions otherwise

On terminal 2:
```
ros2 topic list
```
You should be seeing the topics

