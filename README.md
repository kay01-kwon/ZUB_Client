# ZUB_Client

Terminal 1
```
roscore
```

Run socket_comm_node.

Terminal 2
```
rosrun sock_comm socket_comm_node
```

Test node.

Terminal 3
```
rosrun sock_comm socket_comm_node_test
```

message info (LIFT, PAN, WHEEL)

msg_pkg/target

int16[3] target_torque
int32[3] target_pos
int32[3] target_vel
