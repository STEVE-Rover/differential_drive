#!/usr/bin/env python

# Description: This node takes as input the velocity target of the wheels and converts it to motor values. 
#              It also publishes the encoder values for both sides
# Author: Jeremie Bourque - jeremie.bourque@usherbrooke.ca

import time
import rospy
import threading
from copy import deepcopy
from differential_drive.msg import VelocityTargets, Encoders
from std_msgs.msg import Int32, Float32

class MotorVelocityController:
    def __init__(self):
        rospy.Subscriber("wheel_vel_target", VelocityTargets, self.velocityTargetsCB, queue_size=1)
        rospy.Subscriber("/ros_talon1/current_position", Float32, self.lfwheelCB)
        rospy.Subscriber("/ros_talon2/current_position", Float32, self.rfwheelCB)
        rospy.Subscriber("/ros_talon5/current_position", Float32, self.lbwheelCB)
        rospy.Subscriber("/ros_talon6/current_position", Float32, self.rbwheelCB)
        self.pub_wheel_enc = rospy.Publisher("wheel_enc", Encoders, queue_size=1)
        self.pub_motor_cmd_left = rospy.Publisher("motor_cmd_left", Int32, queue_size=1)
        self.pub_motor_cmd_right = rospy.Publisher("motor_cmd_right", Int32, queue_size=1)
        self.upper_limit = 100
        self.lower_limit = -100
        self.maximum_speed = 0.5  # m/s
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 10)
        self.ticks = 0
        self.encoder_min = rospy.get_param("~encoder_min", -32768)
        self.encoder_max = rospy.get_param("~encoder_max", 32768)
        self.encoder_low_wrap = rospy.get_param("~wheel_low_wrap", (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min)
        self.encoder_high_wrap = rospy.get_param("~wheel_high_wrap", (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min)

        self.raw_enc = [0.0] * 4  # Wheel encoders with limits
        self.prev_raw_enc = [0.0] * 4  # Previous wheel encoders with limits
        self.enc_multipliers = [0.0] * 4  # Number of times the encoders have wrapped around
        self.wrapped_enc = [0.0] * 4  # Wheel encoders with wrap around

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
        left_cmd = Int32()
        right_cmd = Int32()
        left_cmd.data = self.limitValue(-msg.left_wheel_vel_target / self.maximum_speed * self.upper_limit)
        right_cmd.data = self.limitValue(msg.right_wheel_vel_target / self.maximum_speed * self.upper_limit)
        self.pub_motor_cmd_left.publish(left_cmd)
        self.pub_motor_cmd_right.publish(right_cmd)
        self.ticks = 0
    
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

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            # if self.ticks > self.timeout_ticks:
            #     # Stop motors
            #     self.pub_motor_cmd_left.publish(Int32())
            #     self.pub_motor_cmd_right.publish(Int32())
            wheel_enc = Encoders()
            wheel_enc.left_encoder = -(int((self.wrapped_enc[0] + self.wrapped_enc[1])/2))
            wheel_enc.right_encoder = (int((self.wrapped_enc[2] + self.wrapped_enc[3])/2))
            self.pub_wheel_enc.publish(wheel_enc)
            self.ticks += 1
        rate.sleep()
    

if __name__ == '__main__':
    try:
        rospy.init_node('motor_velocity_controller')
        rospy.loginfo("Starting the motor_velocity_controller node")
        motor_velocity_controller = MotorVelocityController()
        motor_velocity_controller.loop()
    except KeyboardInterrupt:
        # Stop motors
        MotorVelocityController.pub_motor_cmd_left.publish(Int32())
        MotorVelocityController.pub_motor_cmd_right.publish(Int32())
        raise
    

