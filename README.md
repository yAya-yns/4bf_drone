# error_code_-11
ROB521

# Simuluation

## Setup: 

### Sources to follow: 
- https://www.youtube.com/watch?v=jBTikChu02E
- https://www.youtube.com/watch?v=rxt0aBnBeJI&t=604s

### Notes:
- Don't USE DOCKER!!!
- ROS version does not need to match.
- Instead of `jMAVSim` in the video, use Gazebo
  - https://dev.px4.io/v1.11_noredirect/en/simulation/ros_interface.html

## Run the simulation: 
3 terminal needed in total:

### Terminal 1:
```
cd 4bf_drone/PX4-Autopilot/
make px4_sitl_default gazebo
```

### Terminal 2:
```
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

### Terminal 3:
```
cd 4bf_drone/PX4-Autopilot/integrationtests/python_src/px4_it/mavros/
python3 mission_test.py MC_mission_box.plan
```
