#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # Simple ROS Publisher example: simple_pub_2.py
    # V.1.0.0.
#----------------------------------------------------------------------------------------

import time
import rospy
import roslib
from std_msgs.msg import String

class simple_pub_2():

    def __init__(self):

        rospy.init_node('simple_pub_2', anonymous=False)
        self.pub = rospy.Publisher('pub_topic_2', String, queue_size=10)
        self.i = 0
        self.rate = rospy.Rate(5)
        self.main()


    def main(self):
        while not rospy.is_shutdown():
            msg = 'ROS: Hello World B: {0}'.format(self.i)
            self.i += 1
            self.pub.publish(msg)
            self.rate.sleep()
        rospy.spin()


if __name__ == '__main__':
    try:
        simple_pub_2()
    except rospy.ROSInterruptException:
        pass
