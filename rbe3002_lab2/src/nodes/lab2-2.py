#!/usr/bin/env python

import math
# import copy
from geometry_msgs.msg import PoseStamped, Twist, Pose, PoseWithCovariance
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf
from tf.transformations import euler_from_quaternion


class Robot:

    def __init__(self):
        """"
        Set up the node here
        """
        rospy.init_node("robot", anonymous=True)
        #properties
        self.px=0
        self.py=0
        self.yaw=0
        self.vel=Twist()

        #Publishers and Subscribers
        self.pub=rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/goal", PoseStamped, self.nav_to_pose)
        rospy.spin() #continue to publish and subscribe


    def nav_to_pose(self, goal):
        # type: (PoseStamped) -> None
        """
        This is a callback function. It should extract data from goal, drive in a striaght line to reach the goal and
        then spin to match the msg goal orientation.
        :param goal: PoseStamped
        :return:
        """

        goalPosX = goal.pose.position.x
        goalPosY = goal.pose.position.y
        quat = goal.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        goal_roll, goal_pitch, goal_yaw = euler_from_quaternion(q)
        #convert the goal yaw into a range of 0 to 2pi
        goalAngle=(goal_yaw+(2*math.pi))%(2*math.pi)
        #current pose
        currPosX=self.px
        currPosY=self.py
        currAngle=self.yaw
        #current path angles based on the goal pose
        path_angle = math.atan2((goalPosY-currPosY),(goalPosX-currPosX))
        pathAngle=(path_angle+(2*math.pi))%(2*math.pi)

        ##rotate to face the path
        self.rotate(pathAngle)
        speed=0.15
        #calculate goal distance
        changeX=abs(currPosX-goalPosX)
        changeY=abs(currPosY-goalPosY)
        distance=math.sqrt(math.pow(changeX,2)+math.pow(changeY,2))
        # drive straight to the goal
        self.drive_straight(speed,distance)
        #rotate to the final angle
        currAngle=self.yaw
        self.rotate(goalAngle)

    def drive_straight(self, speed, distance):
        """
        Make the turtlebot drive straight
        :type speed: float
        :type distance: float
        :param speed: speed to drive
        :param distance: distance to drive
        :return:
        """
        #not moving in y, z or turning
        self.vel.linear.y = 0
        self.vel.linear.z = 0

        #current state
        firstPosX=self.px
        firstPosY=self.py
        distTraveled=0
        #while the robot has not traveled the full distance
        while(distTraveled<distance):
            #drive forward in x-dir at the given speed
            self.vel.linear.x=speed
            #publish speed
            self.pub.publish(self.vel)
            # update from previous state
            currPosX = self.px
            currPosY=self.py
            changeX = abs(currPosX - firstPosX)
            changeY = abs(currPosY - firstPosY)
            distTraveled = math.sqrt(math.pow(changeX, 2) + math.pow(changeY, 2))
        #publish a linear velocity of 0 to stop
        self.vel.linear.x = 0
        self.pub.publish(self.vel)
        # print("Done Driving Straight")


    def rotate(self, angle):
        """
        Rotate in place
        :param angle: angle to rotate
        :return:
        """
        #not rotating about x or z axis
        self.vel.angular.x=0
        self.vel.angular.y = 0
        # print ("ORIGN ANGLE: "+str(angle))
        currYaw=self.yaw
        #calculate the error
        err= abs(angle-currYaw)
        #print statements and sleep function for debugging
        # print("Curr Angle: "+str(currYaw))
        # print("Goal Angle: " +str(angle))
        # print("Error: "+str(err))
        # rospy.sleep(5)

        #while the error is large
        while (err>.1):
            if angle<currYaw:
                #if the desired angle is less than current, turn about Z in neg dir
                self.vel.angular.z = -0.15
            else:
                #desired angle is less than current, turn about Z in pos dir
                self.vel.angular.z = 0.15

            # publish speed
            self.pub.publish(self.vel)
            #update from previous state
            currYaw=(self.yaw+(2*math.pi))%(2*math.pi)
            err = abs(angle - currYaw)
            # print("ERROR: " +str(err))
        # publish speed of zero
        self.vel.angular.z=0
        self.pub.publish(self.vel)
        # print("Done Rotating")
        # print("Curr Angle: "+str(self.yaw))
        # print("Goal Angle: " +str(angle))


    def odom_callback(self, msg):
        """
        update the state of the robot
        :type msg: Odom
        :return:
        """
        self.px=msg.pose.pose.position.x
        self.py=msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        roll, pitch, yaw = euler_from_quaternion(q)
        self.yaw=yaw

if __name__ == '__main__':
    try:
        #create a robot object
        robot = Robot()
    except Exception as err:
        # print(err)
        pass