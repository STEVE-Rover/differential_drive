# differential_drive
This package calculates the odometry of a differential drive robot and publishes an odometry message along with the tf transform of the base link to the odom frame. It also takes twist commands and generates velocity commands for the right and left sides of the robot.

## diff_drive
This is the node that receives the wheel angular positions data to calculate the odometry and also receives the velocity commands and converts them to left and right motor commands
### Subscribed topics
* cmd_vel (geometry_msgs/Twist): Twist commands
* wheel_angles (differential_drive/WheelAngularPositions): Averaged angular positions of the left and right sides of the robot.

### Published topics
* wheel_vel_target (differential_drive/VelocityTargets): Velocity targets for both sides of the robot
* odom (nav_msgs/Odometry): Generated odometry message

### Parameters
* ~rate (float, default: 50): Rate (in Hz) at which the loop executes
* ~base_width (float, default: 0.2): Width between the left wheel to the right wheel
* ~wheel_radius (float, default: 0.3): Radius of the wheels
* ~base_frame_id (string, default: "base_link"): Name of base frame
* ~odom_frame_id (string, default: "odom"): Name of odom frame
* ~publish_tf (bool, default: true): Broadcast tf transform for base_link to odom

## hardware_interface
The purpose of this node is to act as the middle man between the differential drive controller and the encoders and the motor controllers. This is where encoder wrap around should be handled and where the motor commands should be modified to be compatible with your hardware. <br>
In other words, this node is responsible for making this package compatible with your specific hardware. It should be modified, or rewritten to fit your needs.

### Subscribed topics
* wheel_vel_target (differential_drive/VelocityTargets): Velocity targets for both sides of the robot
* TODO: individual wheel position topics. 
### Published topics
* wheel_angles (differential_drive/WheelAngularPositions): Averaged wheel angular positions for the left and right sides of the robot.
* motor_cmd_left (std_msgs/Int32): Left motor velocity commands in rad/s
* motor_cmd_right (std_msgs/Int32): Right motor velocity commands in rad/s
* /zeus/left_front_wheel_velocity_controller/command (std_msgs/Float64): Motor command when parameter `simulation` is set to true. There are also similarly named topics for the other five wheels.

### Parameters
* ~rate (float, default: 50): Rate (in Hz) at which the loop executes
* ~wheel_radius (float, default 0.3): Radius of the wheels in meters
* ~ticks_per_revolution (int, default: 6045): Number of ticks per wheel revolution.
* ~encoder_min (int, default: -32768): Minimum encoder value
* ~encoder_max (int, default: 32768): Maximum encoder value
* ~simulation (bool, default: false): If the robot is run in a simulation

## Improvements
* Make `hardware_interface` a part of `diff_drive` instead of two separate nodes to improve performance.