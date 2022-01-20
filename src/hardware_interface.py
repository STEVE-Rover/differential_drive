#!/usr/bin/env python

import rospy
from math import pi
from copy import deepcopy
from differential_drive.msg import VelocityTargets, WheelAngularPositions
from std_msgs.msg import Float32, Float64, Int32
from sensor_msgs.msg import JointState

class MotorVelocityController:
    def __init__(self):
        self.upper_limit = 100
        self.lower_limit = -100
        self.rate = rospy.get_param("~rate", 50)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.3) 
        self.ticks_per_revolution= float(rospy.get_param("~ticks_per_revolution", 6045))
        self.encoder_min = rospy.get_param("~encoder_min", -32768)
        self.encoder_max = rospy.get_param("~encoder_max", 32768)
        self.encoder_low_wrap = (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min
        self.encoder_high_wrap = (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min
        self.simulation = rospy.get_param("~simulation", False)

        self.raw_enc = [0.0] * 4  # Wheel encoders with limits
        self.prev_raw_enc = [0.0] * 4  # Previous wheel encoders with limits
        self.enc_multipliers = [0.0] * 4  # Number of times the encoders have wrapped around
        self.wrapped_enc = [0.0] * 4  # Wheel encoders with wrap around
        self.wheel_angles = [0.0] * 4  # Wheel angles in radians

        rospy.Subscriber("wheel_vel_target", VelocityTargets, self.velocityTargetsCB, queue_size=1)
        if self.simulation:
            rospy.Subscriber("/zeus/joint_states", JointState, self.jointStateCB)
        else:
            rospy.Subscriber("/ros_talon1/current_position", Float32, self.lfwheelCB)
            rospy.Subscriber("/ros_talon2/current_position", Float32, self.rfwheelCB)
            rospy.Subscriber("/ros_talon5/current_position", Float32, self.lbwheelCB)
            rospy.Subscriber("/ros_talon6/current_position", Float32, self.rbwheelCB)
        
        self.pub_wheel_angular_positions = rospy.Publisher("wheel_angles", WheelAngularPositions, queue_size=1)
        if self.simulation:
            self.pub_lf_cmd = rospy.Publisher("/zeus/left_front_wheel_velocity_controller/command", Float64, queue_size=1)
            self.pub_lm_cmd = rospy.Publisher("/zeus/left_middle_wheel_velocity_controller/command", Float64, queue_size=1)
            self.pub_lr_cmd = rospy.Publisher("/zeus/left_rear_wheel_velocity_controller/command", Float64, queue_size=1)
            self.pub_rf_cmd = rospy.Publisher("/zeus/right_front_wheel_velocity_controller/command", Float64, queue_size=1)
            self.pub_rm_cmd = rospy.Publisher("/zeus/right_middle_wheel_velocity_controller/command", Float64, queue_size=1)
            self.pub_rr_cmd = rospy.Publisher("/zeus/right_rear_wheel_velocity_controller/command", Float64, queue_size=1)
        else:
            self.pub_motor_cmd_left = rospy.Publisher("motor_cmd_left", Int32, queue_size=1)
            self.pub_motor_cmd_right = rospy.Publisher("motor_cmd_right", Int32, queue_size=1)

    def jointStateCB(self, msg):
        self.wheel_angles[0] = msg.position[0]
        self.wheel_angles[1] = msg.position[2]
        self.wheel_angles[2] = msg.position[3]
        self.wheel_angles[3] = msg.position[5]
    def lfwheelCB(self, msg):
        self.raw_enc[0] = msg.data
        self.handleWrapAround(0)
    def lbwheelCB(self, msg):
        self.raw_enc[1] = msg.data
        self.handleWrapAround(1)
    def rfwheelCB(self, msg):
        self.raw_enc[2] = msg.data
        self.handleWrapAround(2)
    def rbwheelCB(self, msg):
        self.raw_enc[3] = msg.data
        self.handleWrapAround(3)

    def velocityTargetsCB(self, msg):
        left_angular_vel = -msg.left_wheel_vel_target / self.wheel_radius  # rad/s
        right_angular_vel = msg.right_wheel_vel_target / self.wheel_radius  # rad/s
        left_angular_vel = left_angular_vel / 8 * 100  # FIXME: temporary until ros_talon accepts rad/s
        right_angular_vel = right_angular_vel / 8 * 100
        self.publishMotorCmds(left_angular_vel, right_angular_vel)
    
    def limitValue(self, value):
        if value > self.upper_limit:
            value = self.upper_limit
        elif value < self.lower_limit:
            value = self.lower_limit
        return value

    def handleWrapAround(self, i):
        if (self.raw_enc[i] < self.encoder_low_wrap and self.prev_raw_enc[i] > self.encoder_high_wrap):
            self.enc_multipliers[i] += 1
        elif (self.raw_enc[i] > self.encoder_high_wrap and self.prev_raw_enc[i] < self.encoder_low_wrap):
            self.enc_multipliers[i] -= 1
        # Calculate encoder values with wrap
        self.wrapped_enc[i] = self.raw_enc[i] + self.enc_multipliers[i] * (self.encoder_max - self.encoder_min)
        self.prev_raw_enc[i] = self.raw_enc[i]

    def publishMotorCmds(self, left, right):
        if self.simulation:
            left_cmd = Float64()
            right_cmd = Float64()
            left_cmd.data = left
            right_cmd.data = right
            self.pub_lf_cmd.publish(left_cmd)
            self.pub_lm_cmd.publish(left_cmd)
            self.pub_lr_cmd.publish(left_cmd)
            self.pub_rf_cmd.publish(right_cmd)
            self.pub_rm_cmd.publish(right_cmd)
            self.pub_rr_cmd.publish(right_cmd)
        else:
            left_cmd = Int32()
            right_cmd = Int32()
            left_cmd.data = left
            right_cmd.data = right
            self.pub_motor_cmd_left.publish(left_cmd)
            self.pub_motor_cmd_right.publish(right_cmd)

    def publishWheelAngularPosition(self):
        wheel_angular_positions = WheelAngularPositions()
        if self.simulation:
            wheel_angular_positions.angle_left = (self.wheel_angles[0] + self.wheel_angles[1])/2
            wheel_angular_positions.angle_right = (self.wheel_angles[2] + self.wheel_angles[3])/2
        else:
            wheel_angular_positions.angle_left = ((self.wrapped_enc[0] + self.wrapped_enc[1])/2) / self.ticks_per_revolution * 2 * pi
            wheel_angular_positions.angle_right = ((self.wrapped_enc[2] + self.wrapped_enc[3])/2) / self.ticks_per_revolution * 2 * pi
        # Average of the motors on the same side
        self.pub_wheel_angular_positions.publish(wheel_angular_positions)


    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.publishWheelAngularPosition()
        rate.sleep()
    

if __name__ == '__main__':
    try:
        rospy.init_node('motor_velocity_controller')
        rospy.loginfo("Starting the motor_velocity_controller node")
        motor_velocity_controller = MotorVelocityController()
        motor_velocity_controller.loop()
    except KeyboardInterrupt:
        # Stop motors
        MotorVelocityController.publishMotorCmds(0, 0)
        raise
    

