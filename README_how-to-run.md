# Step by step guide to run the simulator

After setting up the environment run

```
ros2 launch nav_main gazebo_simulator.launch.py
```

In another terminal run 
```
docker exec -it <container-name> bash
```
if you press the TAB KEY <container-name> should be autocompleted

Once inside run
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

You should see the robot moving on Rviz when sending commands over the /cmd_vel topic
