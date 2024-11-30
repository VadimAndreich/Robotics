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

From ~/my_robot/robot_description/urdf:
```
ros2 run xacro xacro robot.urdf.xacro -o robot.urdf
```

---

