#!/usr/bin/env python
import rospy
import message_filters
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import gazebo_msgs.msg
import numpy as np
from tf.transformations import quaternion_multiply
from tf.transformations import quaternion_matrix
from geometry_msgs.msg import Point, Quaternion
from tfe_msgs.msg import Odom2DWithCovariance
import tf
import json
import sys


class GimbalMover:
    def __init__(self):
        """
        Initialize the data collector.
        :param topic_name: The name of the ROS topic to subscribe to.
        :param message_type: The type of the ROS message.
        :param output_file: The file path where the data will be saved.
        """
        self.altitude=sys.argv[3]
        self.speed=sys.argv[2]
        self.shape=sys.argv[1]
        self.data = [0,0]
        self.Previous_measurment=0
        self.current_pitch=0
        self.current_roll=0
        self.corr_quat=np.array([0.0, -0.7071, 0.0, 0.7071])
        self.position_updated=1
        self.corrected_position=Point
        self.rec_data = [[],[],[],[]]
        self.output_file = "/home/user/Projects/catkin_ws/position_error_{0}_{1}m_s_{2}m_FIXED.json".format(self.shape, self.speed, self.altitude)
        self.corrected_odom=[]
        self.greatest_ang_vel=0

        # Initialize ROS node (anonymous=True ensures the node has a unique name, avoiding conflicts)
        rospy.init_node('gimbal_mover_node', anonymous=True)
        
        
        # Create publisher for the 'gps/vel' topic
        self.angle_pub = rospy.Publisher('uav1/servo_camera/desired_orientation', Float32MultiArray, queue_size=10)
        self.angle_msg = Float32MultiArray()
        
        
        # Subscribe to topics
        self.target_position = message_filters.Subscriber("/uav/gimbal/Target_Relative_Pose", PoseStamped)
        self.gimbal_orientation = message_filters.Subscriber("/gazebo/link_states", gazebo_msgs.msg.LinkStates)
        self.target_GT = message_filters.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates)
        self.target_global_pos = message_filters.Subscriber('/uav/Target_global_pose_filtered', Odom2DWithCovariance)
        self.UAV_global_pos = message_filters.Subscriber('/uav/UAV_global_pose', Odometry)
        
        self.Target_global_pose_corrected = rospy.Publisher('/uav/Target_global_pose_corrected', Odom2DWithCovariance, queue_size=10) #PoseStamped
        
        #self.sync = message_filters.ApproximateTimeSynchronizer([self.target_position, self.gimbal_orientation, self.target_GT, self.target_global_pos, self.UAV_global_pos], queue_size=10, slop=0.1, allow_headerless=True)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.target_position, self.gimbal_orientation, self.target_GT, self.target_global_pos, self.UAV_global_pos], queue_size=10, slop=0.1, allow_headerless=True)
        self.sync.registerCallback(self.callback)
        
        # Control the duration we want to listen for messages
        self.listen_duration = rospy.spin()

    def correct_target_position(self, gimbal_orientation, target_position):
        # Convert quaternion to rotation matrix
        quaternion = np.array([gimbal_orientation.x, gimbal_orientation.y, gimbal_orientation.z, gimbal_orientation.w])

        
        rotation_matrix = np.array(tf.transformations.quaternion_matrix(quaternion))[:3, :3]

        rotation_matrix = np.array([ [ rotation_matrix[0,0], rotation_matrix[1,0], rotation_matrix[2,1] ] , [ rotation_matrix[0,1], rotation_matrix[1,1], rotation_matrix[2,0]] , [ rotation_matrix[1,2], rotation_matrix[0,2], rotation_matrix[2,2]]])
        
        #print("Corrected gimbal quaternion: ",quaternion)
        
        # Invert the rotation matrix
        rotation_matrix_inv = rotation_matrix.T
        
        #print("Gimbal rotation matrix: \n",rotation_matrix_inv)
        
        #print("Uncorrected target position: \n",target_position)
       

        # Convert target position to numpy array
        target_pos_array = np.array([target_position.x, target_position.y, target_position.z])

        # Apply the inverse rotation to correct the target's position
        corrected_position = np.dot(rotation_matrix_inv, target_pos_array)
        
        # Apply the inverse rotation to correct the target's position
        corrected_position = np.array([corrected_position[0],corrected_position[1],corrected_position[2]])

        # Convert numpy array back to geometry_msgs/Point
        corrected_position_msg = Point(*corrected_position)
        return corrected_position_msg

    def callback(self, target_msg, gimb_orient_msg, target_GT, target_global_pos_msg, UAV_global_pos_msg):
        print("test")
        self.rec_data[0].append(target_msg.header.stamp.secs + target_msg.header.stamp.nsecs/1000000000)
    	
        for i in range(len(target_GT.name)):
            if target_GT.name[i]=="Target_2":
                self.rec_data[1].append([target_GT.pose[i].position.x,target_GT.pose[i].position.y])
    	

        
        data = gimb_orient_msg
        msg = target_msg
        
        for i in range(len(data.name)):
            if data.name[i]=="uav1::servo_camera_link":
                x=data.pose[i].orientation.x
                y=data.pose[i].orientation.y
                z=data.pose[i].orientation.z
                w=data.pose[i].orientation.w
                orient_temp=np.array([x,y,z,w])
                #print("Uncorrected Gimbal: ",orient_temp)
                corrected_orient = quaternion_multiply( orient_temp, self.corr_quat)
                qx=corrected_orient[0]
                qy=corrected_orient[1]
                qz=corrected_orient[2]
                qw=corrected_orient[3]
                gimbal_orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
                gimbal_orientation2 = np.array([qx, qy, qz, qw])
                #print("Corrected Gimbal: \n",gimbal_orientation)
    
    
        
        
        """
        Callback function for the topic subscriber. It collects data from each message.
        :param msg: The received message.
        """
        
        tx = msg.pose.position.x
        ty = msg.pose.position.y
        tz = msg.pose.position.z

        self.rec_data[3].append(math.atan(np.sqrt(tx**2+ty**2)/tz))
        
        #print("Drone view position: \nx= ",tx,"\ny= ",ty,"\nz= ",tz)
        if tz>900:
            self.angle_msg.data = [0,0]
            print("LOST VIEW OF ARUCO CODE")
            self.angle_pub.publish(self.angle_msg)
            self.rec_data[2].append([0, 0])
            with open(self.output_file, 'w') as file:
                json.dump(self.rec_data, file)
            return
        
        
        target_position = Point(x=tx, y=ty, z=tz)
      
        
        self.corrected_position = self.correct_target_position(gimbal_orientation, target_position)

        
        x=self.corrected_position.x+0.04
        y=self.corrected_position.y-0.1
        z=self.corrected_position.z

        
        #print("Drone relative position: \nx= ",x,"\ny= ",y,"\nz= ",z)
        
        
        target_time = target_msg.header.stamp.secs + target_msg.header.stamp.nsecs/1000000000
        uav_time = UAV_global_pos_msg.header.stamp.secs + UAV_global_pos_msg.header.stamp.nsecs/1000000000
        dt = target_time - uav_time
        
        
        measured_pose = UAV_global_pos_msg.pose.pose
        measured_velocity = UAV_global_pos_msg.twist.twist
        
        position_vector = np.array([measured_pose.position.x, measured_pose.position.y, measured_pose.position.z])
        velocity_vector = np.array([measured_velocity.linear.x, measured_velocity.linear.y, measured_velocity.linear.z])
        
        corrected_position = position_vector+ velocity_vector*dt
        
        #xGP=xrot1 - measured_pose.position.x
        #yGP=yrot1 - measured_pose.position.y
        
        current_orientation = np.array([measured_pose.orientation.x,measured_pose.orientation.y,measured_pose.orientation.z,measured_pose.orientation.w])
        current_angular_velocity = np.array([measured_velocity.angular.x,measured_velocity.angular.y,measured_velocity.angular.z])
        corrected_orientation = calculate_quaternion_update(current_orientation, current_angular_velocity, dt)
        

        
        UAV_quat=UAV_global_pos_msg.pose.pose.orientation
        wuav=UAV_quat.w
        xuav=UAV_quat.x
        yuav=UAV_quat.y
        zuav=UAV_quat.z
            
        UAV_quat=np.array([xuav,yuav,zuav,wuav])
        #UAV_quat = corrected_orientation
        UAV_rot_mat = np.array(tf.transformations.quaternion_matrix(UAV_quat))[:3, :3]
        UAV_rot_mat = UAV_rot_mat.T

        
        xt=-y
        yt=-x
        zt=-z
        
        
        #print("World relative position: \nx= ",xt,"\ny= ",yt,"\nz= ",zt)
        

        
        [[xrot1],[yrot1],[zrot1]]=np.dot(UAV_rot_mat, np.array([[xt],[yt],[zt]]))

        

        #print("corrected relative position \nx= ",xrot1,"\ny= ",yrot1,"\nz= ",zrot1)

        xGP=corrected_position[0] + xt#xrot1
        yGP=corrected_position[1] + yt#yrot1

        target_global_pos_msg.pose.x=xGP
        target_global_pos_msg.pose.y=yGP
        print("Corrected position published")
        self.Target_global_pose_corrected.publish(target_global_pos_msg)
        self.rec_data[2].append([xGP,yGP])
        
        #print("Absolute position: \nx= ",xGP,"\ny= ",yGP,"\nz= ",zrot1)
        
        
        #print("corrected UAV orientation: \n",UAV_rot_mat)
        
        xxx=xrot1
        xrot1=-yrot1
        yrot1=-xxx
        zrot1=-zrot1
        
        roll = math.atan(xrot1/zrot1) 
        
        RM=np.array([[math.cos(roll), 0, math.sin(roll)],[0, 1, 0],[-math.sin(roll), 0, math.cos(roll)]]).T #rotation matrix for a rotation around Y-axis and amplitude "roll" radians
        [[xrot2],[yrot2],[zrot2]]=np.dot(RM, np.array([[xrot1],[yrot1],[zrot1]])) #calculate point coordinates in rotated frame of reference
             
        pitch = math.atan(-yrot2/zrot2)

           
           
        self.data[0] = -roll  #gimbal controller takes inverted input for some reason
        self.data[1] = -pitch
        
        limit=1.3
        if self.data[0]>limit:
            self.data[0]=limit
        elif self.data[0]<-limit:
            self.data[0]=-limit
        if self.data[1]>limit:
            self.data[1]=limit
        elif self.data[1]<-limit:
            self.data[1]=-limit
        self.angle_msg.data = self.data
        #self.angle_msg.data = [0.05,-0.05]
        #print("Gimbal input= ",self.data)
        self.angle_pub.publish(self.angle_msg)
        
        
        
        
        
        # T_x = target_global_pos_msg.pose.x #- 18.8 - UAV_global_pos_msg.pose.pose.position.x
        # T_y = target_global_pos_msg.pose.y 
        # T_z = 20
        
        # #print("Global position1: \nx= ", T_x,"\ny= ",T_y)
        
        # rotation_matrix = np.array(tf.transformations.quaternion_matrix(gimbal_orientation2))[:3, :3]
        # rotation_matrix = rotation_matrix.T


        # [[T_x],[T_y],[T_z]] = np.dot(rotation_matrix,[[T_x],[T_y],[T_z]])
        
        # #print("global pose rotation matrix: \n",rotation_matrix)
        
        # target_global_pos_msg.pose.x=T_x
        # target_global_pos_msg.pose.y=T_y
        # #self.Target_global_pose_corrected.publish(target_global_pos_msg)
        
        # #self.rec_data[2].append([x,y])
        # #self.rec_data[2].append([T_x, T_y])
        
        # #print("Global position2: \nx= ", T_x,"\ny= ",T_y)
        

        self.greatest_ang_vel = max(self.greatest_ang_vel, np.sqrt((UAV_global_pos_msg.twist.twist.angular.x)**2+(UAV_global_pos_msg.twist.twist.angular.y)**2+(UAV_global_pos_msg.twist.twist.angular.z)**2))
        print("Max encountered angular velocity :", self.greatest_ang_vel)
        
        
        
        
        
        
        with open(self.output_file, 'w') as file:
            json.dump(self.rec_data, file)
        
    def change_angle(self):
    
        rospy.sleep(self.listen_duration)
        
def calculate_quaternion_update(q, omega, dt):
    """
    Calculate the quaternion update using the given current quaternion q, angular velocity omega,
    and time step dt.
    
    :param q: numpy array of shape (4,1) representing the current quaternion [qw, qx, qy, qz]
    :param omega: numpy array of shape (3,1) representing the angular velocity [wx, wy, wz]
    :param dt: float representing the time step
    
    :return: numpy array of shape (4,1) representing the updated quaternion [qw, qx, qy, qz]
    """

    # Construct the Omega matrix
    omega_matrix = np.array([[0, -omega[0], -omega[1], -omega[2]],
                             [omega[0], 0, omega[2], -omega[1]],
                             [omega[1], -omega[2], 0, omega[0]],
                             [omega[2], omega[1], -omega[0], 0]])

    # Calculate the quaternion update matrix
    update_matrix = np.identity(4) + 0.5 * omega_matrix * dt
    # Calculate the updated quaternion matrix
    q_new = update_matrix@q
    # Convert the updated quaternion matrix back to vector form
    q_new_normalized =  q_new / np.sqrt(np.sum(q_new**2))
    return q_new_normalized
        
if __name__ == '__main__':
    # Customize these variables

    collector = GimbalMover()
    collector.change_angle()
