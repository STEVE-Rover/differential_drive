#!/usr/bin/env python

import rospy
from copy import deepcopy
from math import sin, cos, pi 
from differential_drive.msg import VelocityTargets, WheelAngularPositions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster 
from tf.transformations import quaternion_from_euler

class DifferentialDrive():
    def __init__(self):
        self.w = float(rospy.get_param("~base_width", 0.2))
        self.rate = rospy.get_param("~rate", 10)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 20)  # TODO: change to time or frequency instead of nb of ticks
        self.wheel_radius= float(rospy.get_param("~wheel_radius", 0.3))
        self.base_frame_id = rospy.get_param("~base_frame_id","base_link")
        self.odom_frame_id = rospy.get_param("~odom_frame_id", "odom")
        self.publish_tf = rospy.get_param("~publish_tf", True)

        self.vel_target = VelocityTargets()
        self.prev_wheel_angles = WheelAngularPositions()  # Previous wheel angular positions in radians
        self.prev_wheel_angles.angle_left = None 
        self.prev_wheel_angles.angle_right = None 

        self.odometry = Odometry()
        self.odometry.header.frame_id = self.odom_frame_id
        self.odometry.child_frame_id = self.base_frame_id
        self.odometry.pose.covariance = [1, 0, 0, 0, 0, 0,
                                         0, 1, 0, 0, 0, 0,
                                         0, 0, 1, 0, 0, 0,
                                         0, 0, 0, 1, 0, 0,
                                         0, 0, 0, 0, 1, 0,
                                         0, 0, 0, 0, 0, 1]
        self.odometry.twist.covariance = [1, 0, 0, 0, 0, 0,
                                          0, 1, 0, 0, 0, 0,
                                          0, 0, 1, 0, 0, 0,
                                          0, 0, 0, 1, 0, 0,
                                          0, 0, 0, 0, 1, 0,
                                          0, 0, 0, 0, 0, 1]
        self.orientation_z = 0

        self.pub_vel_target = rospy.Publisher("wheel_vel_target", VelocityTargets, queue_size=1)
        self.pub_odom = rospy.Publisher("odom", Odometry, queue_size=1)
        self.sub_cmd_vel = rospy.Subscriber("cmd_vel", Twist, self.cmdVelCB)
        self.sub_wheel_angles = rospy.Subscriber("wheel_angles", WheelAngularPositions, self.wheelAnglesCB, queue_size=1)
        self.odom_broadcaster = TransformBroadcaster()

        self.ticks_since_target = 0
        self.prev_time = None

    def cmdVelCB(self, msg):
        # Convert twist cmd to wheel target velocity
        if msg.angular.z == 0: 
            # Linear motion only
            self.vel_target.left_wheel_vel_target = msg.linear.x
            self.vel_target.right_wheel_vel_target = msg.linear.x
        elif msg.linear.x == 0:
            # Angular motion only
            self.vel_target.left_wheel_vel_target = -msg.angular.z * self.w / 2
            self.vel_target.right_wheel_vel_target = -self.vel_target.left_wheel_vel_target
        else:
            # Angular and linear motion
            radius = msg.linear.x / msg.angular.z
            self.vel_target.left_wheel_vel_target = msg.angular.z * (radius - (self.w / 2))
            self.vel_target.right_wheel_vel_target = msg.angular.z * (radius + (self.w / 2))
        self.ticks_since_target = 0

    def wheelAnglesCB(self, msg):
        current_time = rospy.Time.now()
        if self.prev_time != None:
            elapsed_time = (current_time - self.prev_time).to_sec()
            self.updateOdometry(msg)
        self.prev_time = current_time

        self.pub_odom.publish(self.odometry)
        if(self.publish_tf):
            self.publishTf()

    def updateOdometry(self, wheel_angles):
        elapsed_time = (rospy.Time(wheel_angles.header.stamp.secs, wheel_angles.header.stamp.nsecs) - \
                        rospy.Time(self.prev_wheel_angles.header.stamp.secs, self.prev_wheel_angles.header.stamp.nsecs)).to_sec()
        if elapsed_time <= 0:
            return
        if self.prev_wheel_angles.angle_left == None or self.prev_wheel_angles.angle_right == None:
            d_left = 0
            d_right = 0
        elif wheel_angles.angle_left == self.prev_wheel_angles.angle_left and \
             wheel_angles.angle_right == self.prev_wheel_angles.angle_right:
            # No change so no need to update the odometry
            return
        else:
            d_left = (wheel_angles.angle_left - self.prev_wheel_angles.angle_left) * self.wheel_radius
            d_right = (wheel_angles.angle_right - self.prev_wheel_angles.angle_right) * self.wheel_radius
            print("d_left=%f" % (d_left))
        self.prev_wheel_angles = deepcopy(wheel_angles)

        # Calculate distance and rotation traveled in the base frame in the x and y axis
        d_distance = (d_left + d_right) / 2
        if d_right - d_left == 0: 
            # Radius is infinite
            d_rotation = 0
            base_dx = d_distance 
            base_dy = 0
        else:
            # Radius is finite
            radius = (self.w / 2) * ((d_left + d_right) / (d_right - d_left))
            if radius == 0:
                d_rotation = (2 * d_right) / self.w
                base_dx = 0
                base_dy = 0
            else:
                d_rotation = d_distance / radius 
                base_dx = radius * sin(d_rotation)
                base_dy = -(radius * (cos(d_rotation) - 1))

        # Transform distances in the odom frame
        odom_dx = base_dx * cos(self.orientation_z) - base_dy * sin(self.orientation_z)
        odom_dy = base_dx * sin(self.orientation_z) + base_dy * cos(self.orientation_z)

        # Calculate velocities
        self.odometry.twist.twist.linear.x = d_distance / elapsed_time
        self.odometry.twist.twist.angular.z = d_rotation / elapsed_time

        # Calculate final pose of the robot
        self.odometry.pose.pose.position.x += odom_dx
        self.odometry.pose.pose.position.y += odom_dy
        self.orientation_z += d_rotation
        quaternion = quaternion_from_euler(0, 0, self.orientation_z)
        self.odometry.pose.pose.orientation.x = quaternion[0]
        self.odometry.pose.pose.orientation.y = quaternion[1]
        self.odometry.pose.pose.orientation.z = quaternion[2]
        self.odometry.pose.pose.orientation.w = quaternion[3]
        self.odometry.header.stamp = rospy.Time.now()

    def publishTf(self):
        self.odom_broadcaster.sendTransform(
                (self.odometry.pose.pose.position.x, self.odometry.pose.pose.position.y, 0),
                (self.odometry.pose.pose.orientation.x, self.odometry.pose.pose.orientation.y,
                 self.odometry.pose.pose.orientation.z, self.odometry.pose.pose.orientation.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
                )

    def loop(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.ticks_since_target > self.timeout_ticks:
                self.vel_target.left_wheel_vel_target = 0
                self.vel_target.right_wheel_vel_target = 0
            self.pub_vel_target.publish(self.vel_target)
            self.ticks_since_target += 1
            r.sleep()
            

if __name__ == "__main__":
    rospy.init_node("diff_drive")
    differential_drive = DifferentialDrive()
    rospy.loginfo("Differential drive ready")
    differential_drive.loop()
    
