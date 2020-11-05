#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import geometry_msgs
import tf

import random
from math import pi
import math

class Pose():
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta


class Driving():
    def __init__(self, pose):
        rospy.init_node('minitask2', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.r = rospy.Rate(10)
        self.pose = pose
        self.range = []
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.default = Twist()
        self.rot_left = Twist()
        self.rot_left.angular.z = 0.2
        self.rot_right = Twist()
        self.rot_right.angular.z = -0.2
        self.rot_left_find_view = Twist()
        self.rot_left_find_view.angular.z = 0.1
        self.fwd = Twist()
        self.fwd.linear.x = 0.2

        # initial -1 means no obstacle found
        self.obstacle_angle = -1

        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
    
    def shutdown(self):
        rospy.loginfo('stopping turtlebot')
        self.pub.publish(Twist())
        rospy.sleep(1)

    def odom_callback(self, msg):
	   # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
								
        self.pose.theta = yaw 
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y	

    def laser_callback(self, data):
        self.range = data.ranges

    @staticmethod
    def random_direction():    
        deg = random.randint(0, 359)
        # left or right, positive rad or negative rad
        if deg >= 181:
            deg = deg - 360
        tar_rad = (pi/180) * deg
        return deg, tar_rad

    def rotate(self, deg, rad):
        if rad == 0:
            self.pub.publish(self.default)
            self.r.sleep()
            print('0 radius got, stay situ')
            return
        elif rad > 0:
            print('%d degree get, turning left for %f radius' % (deg, rad))
        else:
            print('%d degree get, turning right for %f radius, %d degree' % ((deg + 360), rad, deg))

        spin_accumulate = 0
        ori_theta = self.pose.theta

        # if turn right, the rad will be negative
        while spin_accumulate < abs(rad):
            diff = self.pose.theta - ori_theta

            if rad > 0:
                spin_accumulate = spin_accumulate + diff
                ori_theta = self.pose.theta
                self.pub.publish(self.rot_left)
                self.r.sleep()
                
                # when turning left, diff keeps positive, if it suddenly become
                # negative, means turning left beyond pi
                if diff < 0:
                    break

            else:
                spin_accumulate = spin_accumulate - diff
                ori_theta = self.pose.theta
                self.pub.publish(self.rot_right)
                self.r.sleep()

                # when turning right, diff keeps negative, if it suddenly become
                # positive, means turning right beyond pi
                # 1 means tolerance, at the start of program the value i got:
                # for example: 7.30654213827e-06, 3.63826088807e-07
                # when past the threshold, it will suddenly become nearly 6.26 (2pi)
                if diff > 1:
                    break

        self.pub.publish(self.default)
        self.r.sleep()
        return

    def obstacle_detect(self):
        # 60 degrees wide laser window, found by experiment
        # angle 0 to 30
        for i in range(31):
            if self.range[i] < 0.5:
                self.obstacle_angle = i
                print('obstacle detected')
                return False
        # angle 330 to 359
        for j in range(330, 360):
            if self.range[j] < 0.5:
                self.obstacle_angle = j
                print('obstacle detected')
                return False
        print('no obstacle detected')
        return True

    # left rotate until can identify a view that without obstacle
    def no_obstacle_view_detect(self):
        print('a view without obstacle detecting')
        while not self.obstacle_detect():
            self.pub.publish(self.rot_left_find_view)
            self.r.sleep()
        self.pub.publish(self.default)
        self.r.sleep()

    def go_forward(self):
        print('go forward in progress')
        dis_accum = 0
        ori_x = self.pose.x
        ori_y = self.pose.y

        while dis_accum < 3:
            if self.obstacle_detect():
                dis_accum = dis_accum + math.sqrt(pow(self.pose.x-ori_x,2)+pow(self.pose.y-ori_y,2))
                ori_x = self.pose.x
                ori_y = self.pose.y
                self.pub.publish(self.fwd)
                print('forwarding')
                print('dis_accum: %f' % dis_accum)
                self.r.sleep()
            # obstacle found during walking
            else:
                self.no_obstacle_view_detect()
                return False


        self.pub.publish(self.default)
        self.r.sleep()
        print('successfully forwarded for 3 meters')
        return True

    def driving(self):
        while not rospy.is_shutdown():
            # wtf, is that python2.7 can't use * to split tuple? too bad on this
            # make code bad looking (or maybe just because i CODE BAD)
            dir = self.random_direction()
            self.rotate(dir[0], dir[1])

            # if detected obstacle, turn left until no obstacle
            if not self.obstacle_detect():
                self.no_obstacle_view_detect()

            # no obstacle, try to go forward for 3 meters
            # found obstacle during walking
            while not self.go_forward():
                # angle already turned after next forward started
                self.go_forward()


if __name__ == '__main__':
    try:
        run = Driving(Pose(0, 0, 0))
        run.driving()
    except rospy.ROSInterruptException:
        pass
    
    