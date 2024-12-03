## Ex01
Packages:
- robot_description

Files:
- ros2_ws/my_robot/robot_description/urdf/robot.urdf.xacro
- ros2_ws/my_robot/robot_description/urdf/robot.urdf (**Converted from xacro**)
- ros2_ws/src/module05/ex01/my_robot/robot_description/launch/robot_display.launch.py

### How to launch:
```
ros2 launch robot_description robot_display.launch.py
```

### Same but without launch file:
```
ros2 run robot_state_publisher robot_state_publisher <urdf_file>
```

```
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

```
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map bottom_link
```

```
ros2 run rviz2 rviz2 -d <rviz_config_file>
```

### What's happening:
1. robot_state_publisher publishes links' states to the /robot_description topic

2. joint_state_publisher_gui gives ability to control non-fixed joints' states with UI and publishes it to the /joint_states topic

3. static_transform_publisher gives tranformation from bottom_link to map frame for RViz correct work

4. rviz2 opens given config. In this case it's configured to read /robot_description topic

---
## Ex02

### How to convert .xacro to .urdf:
```
ros2 run xacro xacro path/to/your_robot.xacro -o path/to/your_robot.urdf
```
Or:
```
xacro path/to/your_robot.xacro > path/to/your_robot.urdf
```

From ~/my_robot/robot_description/urdf:
```
xacro robot.urdf.xacro > robot.urdf
```

Also, how to convert usdf to sdf(gazebo does it automatically):
```
gz sdf -p robot.urdf > robot.sdf
```

---
## Ex03
How to launch:
```
ros2 launch robot_description robot_gazebo.launch.py
```

How to control:
```
ros2 topic pub /robot/cmd_vel geometry_msgs/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

Or:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot/cmd_vel
```

Else:
```
ros2 run rqt_robot_steering rqt_robot_steering
```

---
## Ex04
Packages:
- move_around

Files:
- launch/circle_movement.launch.py
- move_around/circle_movement.py

### How to launch:
```
ros2 launch move_around circle_movement.launch.py
```

And from another terminal:
```
ros2 run move_around movements_node
```

### What's happening:
circle_movement.py spins a node, that publishes to /robot/cmd_vel with linear speed x = 0.5 and angular speed z = 0.5 making robot spin

---
## Ex05
Packages:
- pattern_movement

Files:
- launch/pattern_movement.launch.py
- pattern_movement/pattern_movement.py

### How to launch:
```
ros2 launch pattern_movement pattern_movement.launch.py
```

And from another terminal:
```
ros2 run pattern_movement movements_node
```

### What's happening:
pattern_movement.py spins a node, that publishes to /robot/cmd_vel with linear speed x = 1.0 and, periodically, angular speed z = 0.5 making robot spin for ~90 degrees, making a rounded square






