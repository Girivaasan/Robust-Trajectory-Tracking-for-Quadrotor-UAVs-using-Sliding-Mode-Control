# Robust-Trajectory-Tracking-for-Quadrotor-UAVs-using-Sliding-Mode-Control

## Follow these commands
```
mkdir -p ~/quad_control_ws/src
cd ~/quad_control_ws/src
catkin_init_workspace 
cd ~/quad_control_ws
catkin init
cd ~/quad_control_ws/src
git clone -b dev/ros-noetic https://github.com/gsilano/CrazyS.git
git clone -b med18_gazebo9 https://github.com/gsilano/mav_comm.git
git clone https://github.com/Girivaasan/Robust-Trajectory-Tracking-for-Quadrotor-UAVs-using-Sliding-Mode-Control.git

```

## Running the controller
```
roslaunch rotors_gazebo crazyflie2_without_controller.launch
```
## Tracked Trajectory

Desired waypoints to be tracked: (0,0,0) --> (0,0,1) --> (1,0,1) --> (1,1,1) --> (0,1,1) --> (0,0,1) in 65 seconds

## Plot for desired trajectory
![Screenshot from 2022-12-25 14-14-27](https://user-images.githubusercontent.com/118299474/209479694-12ccd820-0cfd-4e83-aee4-9c4d9df04866.png)

## Performance Trajectory plot from Gazebo
![Figure_1](https://user-images.githubusercontent.com/118299474/209479712-c7d2dceb-72ad-424a-9d0c-6054e34e8883.png)



https://user-images.githubusercontent.com/118299474/209479916-c20df44d-acac-4497-9a10-23c86f2192b9.mp4

