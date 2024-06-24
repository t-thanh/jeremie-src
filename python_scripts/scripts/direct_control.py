#!/usr/bin/env python
import rospy
import message_filters
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import gazebo_msgs.msg
import numpy as np
from tf.transformations import quaternion_multiply
from tf.transformations import quaternion_matrix
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry
import tf
import subprocess


class direct_controller:
    def __init__(self):
        """
        Initialize the data collector.
        :param topic_name: The name of the ROS topic to subscribe to.
        :param message_type: The type of the ROS message.
        :param output_file: The file path where the data will be saved.
        """
        
        
        self.data = [0,0]
        self.current_pitch=0
        self.current_roll=0
        self.corr_quat=np.array([-0.005, -0.7071, 0.005, 0.707])
        self.position_updated=1
        self.corrected_position=Point
      

        # Initialize ROS node (anonymous=True ensures the node has a unique name, avoiding conflicts)
        rospy.init_node('direct_controller_node', anonymous=True)
        
        
        # Subscribe to topics
        #self.target_position = message_filters.Subscriber("/uav/gimbal/Target_Relative_Pose", PoseStamped)
        self.target_position = message_filters.Subscriber("uav/Target_Relative_Pose", PoseStamped)
        self.gimbal_orientation = message_filters.Subscriber("/gazebo/link_states", gazebo_msgs.msg.LinkStates)
        self.UAV_position = message_filters.Subscriber("/uav1/odometry/odom_main", Odometry)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.target_position, self.gimbal_orientation, self.UAV_position], queue_size=10, slop=0.1, allow_headerless=True)
        self.sync.registerCallback(self.callback)
        
        # Control the duration we want to listen for messages
        self.listen_duration = rospy.spin()

    def correct_target_position(self, gimbal_orientation, target_position):
        # Convert quaternion to rotation matrix
        quaternion = [gimbal_orientation.x, gimbal_orientation.y, gimbal_orientation.z, gimbal_orientation.w]
        rotation_matrix = np.array(tf.transformations.quaternion_matrix(quaternion))[:3, :3]
        
        print("Corrected gimbal quaternion: ",quaternion)
        print("Gimbal rotation matrix: \n",rotation_matrix)
        
        # Invert the rotation matrix
        rotation_matrix_inv = rotation_matrix.T
        
        print("Uncorrected target position: \n",target_position)
       

        # Convert target position to numpy array
        target_pos_array = np.array([target_position.x, target_position.y, target_position.z])

        # Apply the inverse rotation to correct the target's position
        corrected_position = np.dot(rotation_matrix_inv, target_pos_array)
        
        # Apply the inverse rotation to correct the target's position
        corrected_position = np.array([corrected_position[0],corrected_position[1],corrected_position[2]])

        # Convert numpy array back to geometry_msgs/Point
        corrected_position_msg = Point(*corrected_position)
        return corrected_position_msg

    def callback(self, target_msg, gimb_orient_msg, UAV_msg):
    	
        print(" ")
        
        uav_x=UAV_msg.pose.pose.position.x
        uav_y=UAV_msg.pose.pose.position.y
        uav_z=UAV_msg.pose.pose.position.z
        
        data = gimb_orient_msg
        msg = target_msg
        
        for i in range(len(data.name)):
            if data.name[i]=="uav1::servo_camera_link":
                x=data.pose[i].orientation.x
                y=data.pose[i].orientation.y
                z=data.pose[i].orientation.z
                w=data.pose[i].orientation.w
                orient_temp=np.array([x,y,z,w])
                print("Uncorrected Gimbal: ",orient_temp)
                corrected_orient = quaternion_multiply(self.corr_quat,orient_temp)
                qx=corrected_orient[0]
                qy=corrected_orient[1]
                qz=corrected_orient[2]
                qw=corrected_orient[3]
                gimbal_orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    
        """
        Callback function for the topic subscriber. It collects data from each message.
        :param msg: The received message.
        """
        
        tx = msg.pose.position.x
        ty = msg.pose.position.y
        tz = msg.pose.position.z
        
        print("uav position: ",uav_x,uav_y, uav_z)
        print("Target position: ",tx, ty, tz)
        absolute_target_pos=[uav_x-ty, uav_y - tx, 20, 0]
        print("Abs position: ", absolute_target_pos)
        
        if tz>900:
            print("LOST VIEW OF ARUCO CODE")
            return
        
        target_position = Point(x=tx, y=ty, z=tz)
      
        
        self.corrected_position = self.correct_target_position(gimbal_orientation, target_position)
        
        print("Corrected: \n",self.corrected_position)
        
        x=self.corrected_position.x
        y=self.corrected_position.y
        z=self.corrected_position.z
        
        #absolute_target_pos=[uav_x-y, uav_y - x, 20, 0]
        # Define the bash command
        bash_command = "rosservice call /uav1/control_manager/goto '"+str(absolute_target_pos)+"'"
        
        subprocess.run(bash_command, shell=True)
        print("Going to ",absolute_target_pos)
        
    def change_angle(self):
        rospy.sleep(self.listen_duration)
        
        
if __name__ == '__main__':
    # Customize these variables
    collector = direct_controller()
    collector.change_angle()
        

# Example usage
#gimbal_orientation = Quaternion(x=0.1630939, y=-0.595689, z=-0.113822, w=0.77800846)  # Example quaternion
#target_position = Point(x=1, y=2, z=3)  # Example target position
#corrected_position = GimbalMover.correct_target_position(gimbal_orientation, target_position)
#print(corrected_position)
