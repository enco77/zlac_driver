#!/usr/bin/env python3.8
import math
import time
import rospy
from zlac_driver.msg import TicksStamped
from nav_msgs.msg import Odometry
from tf import TransformBroadcaster
from geometry_msgs.msg import Quaternion, TransformStamped

class ZLACOdometry():
    def __init__(self) -> None:
        self.ppr = rospy.get_param('ppr', default=1024)
        self.wheel_radius = rospy.get_param('wheel_radius', default=0.085)
        self.wheelbase = rospy.get_param('wheelbase', default=0.28)
        self.publish_tf = rospy.get_param('publish_tf', default = False)
        self.odom_frame = rospy.get_param('odom_frame', default = 'odom')
        self.base_frame = rospy.get_param('base_frame', default = 'base_link')

        self.last_time = rospy.Time.now()
        self.ticks_meter = (self.ppr * 4) / (2 * math.pi * self.wheel_radius)
        self.init = False
        self.enc_left = 0
        self.enc_right = 0
        self.x_final = 0
        self.y_final = 0
        self.theta_final = 0

        rospy.Subscriber('encoder/ticks', TicksStamped, self.encoder_callback)
        self.odom_publisher = rospy.Publisher('odometry', Odometry, queue_size=10)

    def encoder_callback(self, msg:TicksStamped):
        self.current_time = rospy.Time.now()
        left = -msg.ticks.L_tick
        right = msg.ticks.R_tick

        if (self.init == False):
            self.enc_left = left
            self.enc_right = right
            self.init = True

        d_left = (left - self.enc_left) / self.ticks_meter
        d_right = (right - self.enc_right) / self.ticks_meter

        self.enc_left = left
        self.enc_right = right

        d = (d_left + d_right) / 2.0
        th = (d_right - d_left) / self.wheelbase

        elapsed = self.current_time.to_sec() - self.last_time.to_sec()
        dx = d / elapsed
        dr = th / elapsed

        x = math.cos(th) * d
        y = math.sin(th) * d

        self.x_final = self.x_final + ( math.cos(self.theta_final) * x - math.sin(self.theta_final) * y)
        self.y_final = self.y_final + ( math.sin(self.theta_final) * x + math.cos(self.theta_final) * y)

        self.theta_final = self.theta_final + th

        odom_quat = Quaternion()
        odom_quat.x = 0.0
        odom_quat.y = 0.0
        odom_quat.z = math.sin( self.theta_final / 2)
        odom_quat.w = math.cos( self.theta_final / 2)

        if(self.publish_tf):
            odom_trans = TransformStamped()
            odom_broadcaster = TransformBroadcaster()
            odom_trans.header.stamp.nsecs = self.current_time.nsecs
            odom_trans.header.stamp.secs = self.current_time.secs
            odom_trans.header.frame_id = self.odom_frame
            odom_trans.child_frame_id = self.base_frame

            odom_trans.transform.translation.x = self.x_final
            odom_trans.transform.translation.y = self.y_final
            odom_trans.transform.translation.z = 0.0
            odom_trans.transform.rotation = odom_quat

            odom_broadcaster.sendTransformMessage(odom_trans)

        odom = Odometry()
        odom.header.stamp.nsecs = self.current_time.nsecs
        odom.header.stamp.secs = self.current_time.secs
        odom.header.frame_id = self.odom_frame

        odom.pose.pose.position.x = self.x_final
        odom.pose.pose.position.y = self.y_final
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_quat

        odom.child_frame_id = self.base_frame
        odom.twist.twist.linear.x = dx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = dr

        self.odom_publisher.publish(odom)

        self.last_time = self.current_time

if __name__ == '__main__':
    rospy.init_node('zlac_odometry_node', anonymous=True)
    odometry = ZLACOdometry()
    rospy.spin()