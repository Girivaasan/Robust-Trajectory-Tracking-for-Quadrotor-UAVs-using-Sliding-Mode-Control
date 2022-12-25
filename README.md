# Robust-Trajectory-Tracking-for-Quadrotor-UAVs-using-Sliding-Mode-Control

## Follow these commands
```
mkdir -p ~/rbe502_project/src
cd ~/rbe502_project/src
catkin_init_workspace 
cd ~/rbe502_project
catkin init
cd ~/rbe502_project/src
git clone -b dev/ros-noetic https://github.com/gsilano/CrazyS.git
git clone -b med18_gazebo9 https://github.com/gsilano/mav_comm.git
git clone https://github.com/Loahit5101/Sliding-Mode-Control-for-Quadrator-Trajectory-Tracking.git
```

## Running the controller
```
rosrun control controller.py
```
## Tracked Trajectory

Desired waypoints to be tracked: (0,0,0)->(0,0,1)->(1,0,1)->(1,1,1)->(0,1,1)->(0,0,1) in 65 seconds

## Plot for desired trajectory
![Screenshot from 2022-12-25 14-14-27](https://user-images.githubusercontent.com/118299474/209479694-12ccd820-0cfd-4e83-aee4-9c4d9df04866.png)
