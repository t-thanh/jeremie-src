#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3Stamped  # Import the message type

def set_constant_speed(speedx, speedy):
    """
    Publishes a constant velocity message to the 'gps/vel' topic.
    :param speed: The constant speed value (m/s) to set for the USV.
    """
    # Initialize ROS node
    rospy.init_node('constant_speed_node', anonymous=True)

    # Create publisher for the 'gps/vel' topic
    vel_pub = rospy.Publisher('gps/vel', Vector3Stamped, queue_size=10)

    # Create a constant velocity message
    constant_vel_msg = Vector3Stamped()

    # Publish the constant velocity message
    accele_rate = rospy.Rate(1)  # Publish rate (0.25 Hz)
    rate = rospy.Rate(10)  # Publish rate (10 Hz)

    constant_vel_msg.vector.x = speedx  # Set the desired constant speed
    constant_vel_msg.vector.y = speedy  # Assuming no lateral movement

    while not rospy.is_shutdown():
        vel_pub.publish(constant_vel_msg)
        rate.sleep()

if __name__ == '__main__':
    # Set the desired constant speed (adjust as needed)
    speedbackward = 0.25  # 1 m/s as an example
    speedforward = 0  # 1 m/s as an example

    try:
        set_constant_speed(speedbackward, speedforward)
    except rospy.ROSInterruptException:
        pass

