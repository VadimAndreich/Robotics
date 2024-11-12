## Ex01
Packages: 
- service_full_name
- service_full_name_inrefaces

Nodes:
- service_name - service node that processes 3 input strings(surname, name, patronymic) from client_name and sends back string of full name
- client_name - client node that sends 3 strings and gets 1 string of full_name, then displays it

Files:
- service_full_name/service_full_name/client_name.py
- service_full_name/service_full_name/service_name.py
- service_full_name_interfaces/srv/SummFullName.srv

### How to launch:
- service node:
```
ros2 run service_full_name service_name
```
- client node:
```
ros2 run service_full_name client_name Perminov Vadim Andreyevich
```

---

## Ex02
Packages: 
- action_turtle_command
- action_turtle_command_interfaces

Nodes: 
- action_turtle_server - service node that processes commands given by client node and sends corresponding commands to /turtle1/cmd_vel
- action_turtle_client - client node that sends commands to the server

Files:
- action_turtle_command/action_turtle_command/action_turtle_server.py
- action_turtle_command/action_turtle_command/action_turtle_client.py
- action_turtle_command_interfaces/action/MessageTurtleCommands.action

### How to launch:
- turtlesim:
```
ros2 run turtlesim turtlesim_node
```

- service node:
```
python3 action_turtle_server.py
```
- client node
```
python3 action_turtle_client.py
```

---

## Ex03
Files:
- bag_files/turtle_cmd_vel/turtle_cmd_vel_0.mcap
- pose_speed_x1.yaml
- pose_speed_x2.yaml

### How to record:
- Run turtlesim:
```
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```
- Record selected topic:
```
ros2 bag record /turtle1/cmd_vel -o turtle_cmd_vel -s mcap
```

### How to play the recording
```
ros2 bag play turtle_cmd_vel -r 1
ros2 bag play turtle_cmd_vel -r 2
```

### How to save transformations to .yaml file
```
ros2 topic echo /turtle1/pose > pose_speed_x1.yaml
```

---

## Ex04
### Full report:
```
ros2 doctor --report >> doctor.txt
```

---

## Ex05
### How to run:
- Run turtlesim:
```
ros2 run turtlesim turtlesim_node
```
- Run the node
```
ros2 run move_to_goal move x y theta
```
