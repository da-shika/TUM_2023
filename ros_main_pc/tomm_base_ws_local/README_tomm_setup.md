## Turn on Robot

1. Connect the power supply cable
2. Turn the main switch inside the robot
3. Flip the computer switch and skin switch on robot back
4. Turn on the UR5 control plannel of right and left arm
5. Setup the robot arms:
  1. Go to initzialization screen
  2. Press start to release the breaks
  3. Press auto to initalize the robot postion (robot will move!)
  4. Use teach button at the plannel back to move the robot to some good position

## Launch the controller

### Connect to the control pc

Open 6 terminals

  ssh -X student@tomm-control
  cd /home/student/ros/workspaces/tomm_base_ws

### On control pc

  roscore 

### Launching the skin (on control pc)

  roslaunch tomm_skin driver_events_right_hand.launch
  e on
  cf on
  store offsets

### Launching the controller

  roslaunch ur_script_manager robot_script_manager_left.launch 

  roslaunch ur_script_manager robot_script_manager_right.launch

  roslaunch tomm_hardware_real tomm_hardware_real.launch

  roslaunch tomm_bringup real_no_mb.launch

### On local pc

Open 4 terminals

  cd ros/workspaces/tomm_base_ws_local

  roslaunch skin_client_real skin_client_real.launch

  roslaunch tomm_bringup rviz.launch

  roslaunch vive_ros vive.launch

  roslaunch vive_teleop teleop.launch

Buttons:

  Left: On/Off teleop behavior (move to home position)

  Right: Enable tracking 

## Launch the camera

Open 1 terminal

  ssh student@tomm-mind
  cd /home/student/ros/workspaces/perception_ws
  
  roslaunch camera_launcher camera.launch
  or
  roslaunch realsense realsense_repub.launch


Topic:

  /camera/rgb/image_raw or /camera/rgb/image_raw/compressed
  /tomm/joint_states
  /tomm/teleop_left_hand/pose
  /tomm/teleop_right_hand/pose
  /tomm/arm_right/hand/right_hand_back/data
  /tomm/arm_left/hand/left_hand_back/data


## Test in simulation

In package tomm_hardware_real:

  tomm_base_ws/src/tomm-robot/tomm_hardware_real/configs/arm

Modify sim / real in side

  right_arm_config.yaml
  left_arm_config.yaml

## Shutdown (robot pcs)

sudo shutdown

## Failure: 

  Stop tracking 
  Kill all controller nodes
  Kill the teleop nodes
