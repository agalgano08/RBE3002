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

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        odom = rospy.Subscriber('/odom', Odometry, self.update_odometry)

        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to)

        self.px = 0
        self.py = 0
        self.pth = 0


        
  

        

        

    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        # REQUIRED CREDIT

        # Sets the speed and angular velocity of the motors.
        while not rospy.is_shutdown():
            vel_msg = Twist()
            vel_msg.linear.x = linear_speed
            vel_msg.angular.z = angular_speed
            self.vel_pub.publish(vel_msg)
            print("Published send_speed")

            
        

    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        # REQUIRED CREDIT
        
        initx =  self.px
        inity =  self.py

        done = False
        while not done and not rospy.is_shutdown():
            vel_msg = Twist()

            #If the robot has not reached the distance update the positions and keep going.
            if(distance > math.sqrt((self.px - initx)**2 + (self.py-inity)**2)):
                vel_msg.linear.x = linear_speed
                self.vel_pub.publish(vel_msg)

                #print(pose.position.y)
                #print(pose.position.x)
                #print(math.sqrt((self.px - initx)**2 + (self.py-inity)**2))
                
                rospy.sleep(0.05)

            #If the Robot has reached the distance then stop the robot and shutdown rospy.
            else:
                vel_msg.linear.x = 0
                self.vel_pub.publish(vel_msg)
                done = True
                rospy.signal_shutdown("Reached Drive") 
            
        

    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        # REQUIRED CREDIT
        pass  # delete this when you implement your code

    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        # REQUIRED CREDIT
        pass  # delete this when you implement your code

    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        # REQUIRED CREDIT
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw


    def run(self):
        
        while not rospy.is_shutdown():
            l = Lab2()
            pose = Odometry()
            l.update_odometry(pose)
            l.drive(.5,0.5)            

            


if __name__ == '__main__':
    l = Lab2()
    l.run()
