## Project Overview:
Control and visualize the real fanuc lrmate200id7l robot arm. 

## Installtion
1. mkdir -p your_workspace/src 

2. cd your_workspace/src
  
3. git clone https://github.com/UpennGitttt/ros_fanuc_arm_control.git

4. cd ..

5. catkin_make

6. source devel/setup.bash

## control real robotarm
1. roslaunch fanuc_bringup fanuc_lrmate200id7l_bringup_real.launch
2. roslaunch fanuc_plan fanuc_pick_place.launch
   
## visualization
1. roslaunch fanuc_lrmate200id7l_moveit_config moveit_planning_execution.launch
2. roslaunch fanuc_plan fanuc_pick_place.launch

![企业微信截图_16921008629863](https://github.com/UpennGitttt/ros_fanuc_arm_control/assets/98191838/c9e7e61c-f272-44d8-ae7d-df2b22bad91e)

   
