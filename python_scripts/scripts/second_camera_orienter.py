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
import tf


class GimbalMover:
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
        self.corr_quat=np.array([-0.01, -0.7071, 0.01, 0.7071])
        self.position_updated=1
        self.corrected_position=Point

        # Initialize ROS node (anonymous=True ensures the node has a unique name, avoiding conflicts)
        rospy.init_node('gimbal_mover_node', anonymous=True)
        
        
        # Create publisher for the 'gps/vel' topic
        self.angle_pub = rospy.Publisher('uav1/servo_camera/desired_orientation', Float32MultiArray, queue_size=10)
        self.angle_msg = Float32MultiArray()
        
        
        # Subscribe to topics
        self.target_position = message_filters.Subscriber("/uav/gimbal/Target_Relative_Pose", PoseStamped)
        self.gimbal_orientation = message_filters.Subscriber("/gazebo/link_states", gazebo_msgs.msg.LinkStates)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.target_position, self.gimbal_orientation], queue_size=10, slop=0.1, allow_headerless=True)
        self.sync.registerCallback(self.callback)
        
        # Control the duration we want to listen for messages
        self.listen_duration = rospy.spin()

    def correct_target_position(self, gimbal_orientation, target_position):
        # Convert quaternion to rotation matrix
        quaternion = np.array([gimbal_orientation.x, gimbal_orientation.y, gimbal_orientation.z, gimbal_orientation.w])

        
        rotation_matrix = np.array(tf.transformations.quaternion_matrix(quaternion))[:3, :3]
        #rotation_matrix = np.array([ [ rotation_matrix[0,0], rotation_matrix[2,1], rotation_matrix[1,0] ] , [ rotation_matrix[1,2], rotation_matrix[1,1], rotation_matrix[2,0]] , [ rotation_matrix[0,1], rotation_matrix[0,2], rotation_matrix[2,2]]])
        rotation_matrix = np.array([ [ rotation_matrix[0,0], rotation_matrix[0,1], rotation_matrix[1,2] ] , [ rotation_matrix[1,0], rotation_matrix[1,1], rotation_matrix[0,2]] , [ rotation_matrix[1,2], rotation_matrix[0,2], rotation_matrix[2,2]]])
        
        print("Corrected gimbal quaternion: ",quaternion)
        
        # Invert the rotation matrix
        rotation_matrix_inv = rotation_matrix#.T
        
        print("Gimbal rotation matrix: \n",rotation_matrix_inv)
        
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

    def callback(self, target_msg, gimb_orient_msg):
    	
        print(" ")
        
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
                corrected_orient = quaternion_multiply( orient_temp, self.corr_quat)
                qx=corrected_orient[0]
                qy=corrected_orient[1]
                qz=corrected_orient[2]
                qw=corrected_orient[3]
                gimbal_orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
                print("Corrected Gimbal: \n",gimbal_orientation)
    
        """
        Callback function for the topic subscriber. It collects data from each message.
        :param msg: The received message.
        """
        
        tx = msg.pose.position.x
        ty = msg.pose.position.y
        tz = msg.pose.position.z
        
        if tz>900:
            self.angle_msg.data = [0,0]
            print("LOST VIEW OF ARUCO CODE")
            self.angle_pub.publish(self.angle_msg)
            return
        
        target_position = Point(x=tx, y=ty, z=tz)
      
        
        self.corrected_position = self.correct_target_position(gimbal_orientation, target_position)
        
        print("Corrected: \n",self.corrected_position)
        
        x=self.corrected_position.x
        y=self.corrected_position.y
        z=self.corrected_position.z
        
        roll = math.atan(-x/z) 
        pitch = math.atan(y/z)
        limit=1.3
           
        self.data[0] = roll
        self.data[1] = pitch
        
        if self.data[0]>limit: #anti wind-up
            self.data[0]=limit
        elif self.data[0]<-limit:
            self.data[0]=-limit
        if self.data[1]>limit:
            self.data[1]=limit
        elif self.data[1]<-limit:
            self.data[1]=-limit
        
        self.angle_msg.data = self.data
        #self.angle_msg.data = [0.05,-0.05]
        print("Gimbal input= ",self.data)
        self.angle_pub.publish(self.angle_msg)
        
    def change_angle(self):
    
        rospy.sleep(self.listen_duration)
        
        
if __name__ == '__main__':
    # Customize these variables

    collector = GimbalMover()
    collector.change_angle()
        

# Example usage
#gimbal_orientation = Quaternion(x=0.1630939, y=-0.595689, z=-0.113822, w=0.77800846)  # Example quaternion
#target_position = Point(x=1, y=2, z=3)  # Example target position
#corrected_position = GimbalMover.correct_target_position(gimbal_orientation, target_position)
#print(corrected_position)
