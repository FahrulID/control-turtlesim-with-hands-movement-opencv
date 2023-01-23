#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class publisher():
    def __init__(self):
        self.msg = ""
        self.pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.init_node('talker', anonymous=True)

    def talk(self, data):
        if not rospy.is_shutdown():
            msg = "%s" % data
            rospy.loginfo(msg)
            self.pub.publish(msg)
            print("Message sent")

if __name__ == '__main__':
    try:
        tes = publisher()
    except rospy.ROSInterruptException:
        pass