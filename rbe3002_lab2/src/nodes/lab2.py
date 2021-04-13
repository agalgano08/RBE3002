#!/usr/bin/env python2

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist, Pose

from tf.transformations import euler_from_quaternion
import math


class Lab2:

    def __init__(self):
        """
        Class constructor
        """
        # REQUIRED CREDIT
        # Initialize node, name it 'lab2'
        rospy.init_node('lab2', anonymous=True)

        # Initialize the Twist Publisher
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Initialize the Odometry Subscriber
        odom = rospy.Subscriber('/odom', Odometry, self.update_odometry)

        # Initialize the PoseStamped Subscriber
        goalGoTo = rospy.Subscriber(
            '/move_base_simple/goal', PoseStamped, self.go_to)

        # Initialize the robot x,y, and yaw.
        self.px = 0
        self.py = 0
        self.pth = 0

        rospy.sleep(1)

    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        # REQUIRED CREDIT

        # Sets the speed and angular velocity of the motors.
        vel_msg = Twist()
        vel_msg.linear.x = linear_speed
        vel_msg.angular.z = angular_speed
        self.vel_pub.publish(vel_msg)

    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        # REQUIRED CREDIT

        # Get position of the robot
        initx = self.px
        inity = self.py

        # Send value to motors.
        self.send_speed(linear_speed, 0)

        # If the robot has not reached the distance and is under the tolerance sleep a little.
        while(abs(distance - (math.sqrt((self.px - initx)**2 + (self.py-inity)**2))) > 0.01):
            rospy.sleep(0.01)

        # If the Robot has reached the distance then stop the robot.
        self.send_speed(0, 0)

    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        # REQUIRED CREDIT

        # Inital angle
        initial = self.pth

        # Desired angle
        final = angle+self.pth

        # Converted to range of pi to -pi
        final = ((final + math.pi) % (2*math.pi)) - math.pi

        # Amount needed to rotate
        goal = self.calculate(initial, final)

        # Determine which direction to turn
        if goal > 0:
            self.send_speed(0, abs(aspeed))
        else:
            self.send_speed(0, -abs(aspeed))

        # Wait for robot to turn within tolerance.
        while abs(self.calculate(self.pth, final)) > 0.005:
            rospy.sleep(0.01)

        self.send_speed(0, 0)

    def calculate(self, initial, final):
        """
        Calcuates the amount the robot needs to turn.
        :param initial       [float] [rad]   The inital robot angle.
        :param final         [float] [rad]   The desired robot angle.
        """

        # Determine the amount the robot needs to turn.
        if abs(final - initial) <= math.pi:
            return final-initial
        else:
            if(initial < final):
                return final - (initial + 2*math.pi)
            else:
                return (final + 2*math.pi) - initial

    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        # REQUIRED CREDIT

        # Get the desired destionation position.
        x = msg.pose.position.x
        y = msg.pose.position.y

        # Calculate the distance the robot needs to travel.
        distance = math.sqrt(math.pow((x - self.px), 2) +
                             math.pow((y - self.py), 2))

        # Calculate the relative coordinate system for x and y from the robots location.
        newx = x - self.px
        newy = y - self.py

        # Calculate the first turn.
        firstTurn = self.calculate(self.pth, math.atan2(newy, newx))

        self.rotate(firstTurn, 0.15)
        rospy.sleep(1)
        self.drive(distance, 0.1)
        rospy.sleep(.25)

        # Calculate the roll, pitch, and yaw of the final orientation.
        quat_orig = msg.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)

        # Rotate the final amount.
        self.rotate(self.calculate(self.pth, yaw), 0.15)

    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        # Get the robot position.
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y

        # Calculate the roll, pitch, and yaw of the robot orientation.
        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    l = Lab2()
    l.run()
