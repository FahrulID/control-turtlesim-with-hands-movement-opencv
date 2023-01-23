#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from math import pi
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

class subscriber():
    def __init__(self):
        self.turtle = turtle()
        self.rate = rospy.Rate(10) # 10hz

        self.speed = 0
        self.forward = 0
        self.belok = 0

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "%s", data.data)

        msg = data.data
        msg_splitted = msg.split(";")
        self.speed = float(msg_splitted[0])
        self.forward = int(msg_splitted[1])
        self.belok = int(msg_splitted[2])

    def moveTurtle(self):
        while not rospy.is_shutdown():
            self.turtle.move(self.speed, self.forward, self.belok)
            self.rate.sleep()
        
    def listener(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        # rospy.init_node('listener', anonymous=True)

        rospy.Subscriber("chatter", String, self.callback)
        self.moveTurtle()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


class turtle():
    def __init__(self):
        print("Turtle Movement initiated")

        rospy.init_node('robot_cleaner', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.vel_msg = Twist()

    def move(self, speed, forward, belok):        
        if(forward > 0):
            self.vel_msg.linear.x = speed
        elif(forward < 0):
            self.vel_msg.linear.x = -speed
        else:
            self.vel_msg.linear.x = 0

        if(belok > 0):
            self.vel_msg.angular.z = 1
        elif(belok < 0):
            self.vel_msg.angular.z = -1
        else:
            self.vel_msg.angular.z = 0

        #Since we are moving just in x-axis
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
            
        self.velocity_publisher.publish(self.vel_msg)

if __name__ == '__main__':
    subs = subscriber()
    subs.listener()