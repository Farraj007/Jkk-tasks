# This task is for University Research Team 
# JKK-SZE

## Visualizing Car Parameters

##### The attached code is for visualizing car parameters such as reference speed, current speed, reference steering angle, and the current angle.

### Clone the Repository

### Clone this repository into the `Ros2_ws/src` directory:

```bash
git clone https://github.com/Farraj007/Jkk-task.git
```
# Install Requirements
### Install the requirements file:
```bash
pip install -r requirments.txt
```
# Reminder: Environment Setup
### Feel free to add these dependencies to your environment.

# Publish ROS Topics
### To mimic some data from the ROS simulation, publish topics using the following commands in separate terminals:

```bash
ros2 topic pub /lexus3/pacmod/vehicle_speed_rpt pacmod3_msgs/msg/VehicleSpeedRpt "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, vehicle_speed: 0.1, vehicle_speed_valid: true}" 

ros2 topic pub /lexus3/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 5.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}' 

ros2 topic pub /lexus3/pacmod/steering_aux_rpt pacmod3_msgs/msg/SteeringCmd '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "frame"}, enable: true, ignore_overrides: false, clear_override: false, command: 0.5, rotation_rate: 1.0}' 

ros2 topic pub --once /lexus3/pacmod/steering_cmd pacmod3_msgs/msg/SteeringAuxRpt '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "frame"}, steering_torque: 0.5, rotation_rate: 1.0, operator_interaction: false, rotation_rate_sign: false, vehicle_angle_calib_status: true, steering_limiting_active: false, steering_torque_avail: true, rotation_rate_avail: true, operator_interaction_avail: true, rotation_rate_sign_avail: true, vehicle_angle_calib_status_avail: true, steering_limiting_active_avail: true}' 

ros2 topic pub /lexus3/pacmod/enabled std_msgs/msg/Bool '{data: true}'
```
# Build and Run
#### Now you are good to go. Execute the following commands:

```bash
colcon build
```

# Source setup file depending on your shell
```bash
source install/setup.bash
```
```bash
ros2 run task task
```