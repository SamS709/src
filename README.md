# README.md

## Ros2 package for rasptank robot (in developpement):
<ul>
  <li>
    DiffDrive Controller implemented from ros2_control (adapted to rasptank hardware)
  </li>
  <li>
    Face recognition with OpenCV
  </li>
  <li>
    Object detection with ultrasonic sensor
  </li>
</ul>

## Run commands

### On the robot (Pi)

```bash
source install/setup.bash
ros2 launch rasptank_bringup pi_launch.py
```

### On the computer

```bash
source install/setup.bash
ros2 launch rasptank_bringup computer_launch.py
```

#### Teleop toggle (computer)

```bash
# Disable keyboard teleop if needed
ros2 launch rasptank_bringup computer_launch.py teleop:=false
```

#### Networking prerequisites (robot <-> computer)

```bash
# On BOTH robot and computer: set the same ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0

# Optional: pin DDS to Cyclone if needed
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Run the Enrico paper controller (odom -> cmd_vel)

After you have started the bringup launches (`pi_launch.py` on the robot and `computer_launch.py` on your PC), run the controller node. Choose to run it on the Pi or on the computer depending on where you want the controller to execute.

### On the robot (Pi)

```bash
source install/setup.bash
ros2 run rasptank_enrico_paper rasptank_enrico_paper_node
```

### On the computer (with odom remapped if needed)

If the odometry topic is namespaced by the controller, remap `odom` when running:

```bash
source install/setup.bash
ros2 run rasptank_enrico_paper rasptank_enrico_paper_node --ros-args -r odom:=/diffbot_base_controller/odom
```

### Passing parameters at runtime

You can override the goal and gains with `--ros-args -p`:

```bash
ros2 run rasptank_enrico_paper rasptank_enrico_paper_node --ros-args -p goal_x:=1.0 -p goal_y:=0.0
```

If the node does not respond or you don't see `/cmd_vel` being published, check that the `diffbot_base_controller` is active and that `/odom` is available (or remap it to the correct topic).

