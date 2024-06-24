#!/usr/bin/env python
import rospy
import time
import os
import sys

def delayed_launch():
    rospy.init_node('delayed_launcher', anonymous=True)
    rospy.loginfo("Waiting 15 seconds before launching core.launch")
    time.sleep(15)  # Delay for 15 seconds
    altitude = sys.argv[1]
    shape = sys.argv[2]
    speed = sys.argv[3]

    os.system("roslaunch mrs_uav_general automated_core.launch altitude:={0} shape:={1} speed:={2}".format(altitude,shape,speed))

if __name__ == '__main__':
    try:
        delayed_launch()
    except rospy.ROSInterruptException:
        pass

