#!/usr/bin/env python
import rospy
import time
import os
import sys

def delayed_launch():
    rospy.init_node('delayed_launcher', anonymous=True)
    rospy.loginfo("Waiting 15 seconds before launching core.launch")
    time.sleep(15)  # Delay for 15 seconds
    altitude=sys.argv[1]
    os.system("roslaunch mrs_uav_general core.launch altitude:={0}".format(altitude))

if __name__ == '__main__':
    try:
        delayed_launch()
    except rospy.ROSInterruptException:
        pass

