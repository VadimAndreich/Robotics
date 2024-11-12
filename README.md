## Ex02
Package: learning_tf2_py
Files:
- learning_tf2_py/learning_tf2_py/rotating_carrot.py
- learning_tf2_py/launch/rotating_carrot.launch.py
- learning_tf2_py/package.xml
- learning_tf2_py/setup.py

### Parameters of launch:
- 'target_frame' - string, id of target frame that will be followed by turtle2
- 'radius' - float, distance between turtle1 and carrot1
- 'direction of rotation' - int, Direction in which carrot1 is rotating(1 for clockwise, -1 for counter-clockwise.)

### How to launch:
'''
ros2 launch learning_tf2_py rotating_carrot.launch.py target_frame:=carrot1 radius:=2.0 direction_of_rotation:=1
'''

### RViz config
#### Location:
learning_tf2_py/config/carrot.rviz

#### How to open in rviz(from ros2_ws):
'''
ros2 run rviz2 rviz2 -d src/module04/ex02/learning_tf2_py/config/carrot.rviz
'''
