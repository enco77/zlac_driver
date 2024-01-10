#!/usr/bin/env python3.8
from ZLAC8015D import Controller
import math
import rospy
from geometry_msgs.msg import Twist
from zlac_driver.msg import TicksStamped


class Zlac_Driver():
    def __init__(self) -> None:
        self.port = rospy.get_param('port', default='/dev/RS485')
        self.freq = rospy.get_param('freq', default=30)
        self.ppr = rospy.get_param('ppr', default=1024)
        self.wheel_radius = rospy.get_param('wheel_radius', default=0.085)
        self.wheelbase = rospy.get_param('wheelbase', default=0.28)
        self.control_mode = rospy.get_param('control_mode', default=3)
        self.accel_time = rospy.get_param('accel_time', default=1000)
        self.decel_time = rospy.get_param('decel_time', default=1000)
        self.driver_id = rospy.get_param('driver_id', default=1)
        self.max_rpm = rospy.get_param('max_rpm', default=260)

        self.rate = rospy.Rate(self.freq)
        self.driver = Controller(self.port, self.wheel_radius, self.driver_id)

        rospy.Subscriber('cmd_vel', Twist, self.cmd_callback)
        self.ticks_publisher = rospy.Publisher('encoder/ticks', TicksStamped, queue_size=10)
        self.ticks_seq = 0
        self.fault_seq = 0

    def configure_driver(self):
        self.driver.disable_motor()
        self.driver.set_accel_time(self.accel_time, self.accel_time)
        self.driver.set_decel_time(self.decel_time, self.decel_time)
        self.driver.set_mode(self.control_mode)

    def enable_driver(self):
        self.driver.enable_motor()
        self.driver.set_rpm(0,0)

    def disable_driver(self):
        self.driver.set_rpm(0,0)
        self.driver.disable_motor()

    def cmd_callback(self, cmd_vel:Twist):
        left_speed_w = self.calculate_left_speed(cmd_vel.linear.x, cmd_vel.angular.z)
        right_speed_w = self.calculate_right_speed(cmd_vel.linear.x, cmd_vel.angular.z)
        left_speed_rpm = self.to_rpm(left_speed_w)
        right_speed_rpm = self.to_rpm(right_speed_w)
        self.driver.set_rpm(left_speed_rpm, right_speed_rpm)

    def calculate_right_speed(self, x, z):
        return (2 * x - z * self.wheelbase) / (2 * self.wheel_radius)

    def calculate_left_speed(self, x, z):
        return (2 * x + z * self.wheelbase) / (2 * self.wheel_radius)

    def to_rpm(self, value):
        return self.max_limit((value * 60) / (2 * math.pi))

    def max_limit(self, speed):
        if (speed > 0):
            return min(self.max_rpm, speed)
        else:
            return max(-self.max_rpm, speed)

    def pub_encoder_ticks(self):
        ticks = TicksStamped()
        current_time = rospy.Time.now()
        ticks.header.stamp.nsecs = current_time.nsecs
        ticks.header.stamp.secs = current_time.secs
        ticks.header.seq = self.ticks_seq
        ticks.ticks.L_tick, ticks.ticks.R_tick = self.driver.get_wheels_tick()
        self.ticks_publisher.publish(ticks)
        self.ticks_seq += 1

    def pub_driver_status(self):
        status = self.driver.get_fault_code()
        rospy.loginfo(f'Status {status}')

if __name__ == '__main__':
    rospy.init_node('zlac_driver_node', anonymous=True)
    zlac_md = Zlac_Driver()
    zlac_md.configure_driver()
    zlac_md.enable_driver()
    while not rospy.is_shutdown():
        try:
            zlac_md.pub_encoder_ticks()
            # zlac_md.pub_driver_status()
            zlac_md.rate.sleep()
        except rospy.ROSInterruptException:
            zlac_md.disable_driver()