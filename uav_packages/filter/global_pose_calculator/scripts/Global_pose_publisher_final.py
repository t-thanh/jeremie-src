#!/usr/bin/python3


import rospy
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from tfe_msgs.msg import Odom2DWithCovariance

import message_filters 
from tf.transformations import quaternion_from_matrix, quaternion_about_axis
from tf.transformations import euler_from_quaternion
import tf2_ros
import tf2_geometry_msgs
import message_filters
from ukf_bis import UKF
#import csv
#import signal
#import itertools

#from filterpy.kalman import KalmanFilter

a = Odom2DWithCovariance()

print("Imports done") #REMOVE

####################################################################################
# Function for the transformation between the camera frame and the global frame
####################################################################################

def transform_pose_to_fcu(posestamped_msg,static_transform):
    # Transform the posestamped_msg to the uav1/fcu frame using the static transform
    transformed_pose = tf2_geometry_msgs.do_transform_pose(posestamped_msg, static_transform)

    # Copy the timestamp from the input message to the output message
    transformed_pose.header.stamp = posestamped_msg.header.stamp

    return transformed_pose


def transform_to_local_origin(pose_stamped_msg, transform_msg):
    # Transform the pose to the new frame
    transformed_pose_stamped = tf2_geometry_msgs.do_transform_pose(pose_stamped_msg, transform_msg)
    
    # Set the header timestamp to match the original message
    transformed_pose_stamped.header.stamp = pose_stamped_msg.header.stamp

    return transformed_pose_stamped

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


def corrected_odometry(odometry_msg, dt):
    """
    Create a geometry msg of the position of the drone at the time of the video recording
    """
    # Get current pose and velocity

    current_pose = odometry_msg.pose.pose
    current_velocity = odometry_msg.twist.twist.linear
    
    # Calculate the position correction
    position_vector = np.array([current_pose.position.x,current_pose.position.y,current_pose.position.z])
    velocity_vector = np.array([current_velocity.x,current_velocity.y,current_velocity.z])

    # Apply position correction
    corrected_position = position_vector+ velocity_vector*dt

    # Calculate the orientation correction
    current_orientation = np.array([current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w])
    current_angular_velocity = np.array([odometry_msg.twist.twist.angular.x,odometry_msg.twist.twist.angular.y,odometry_msg.twist.twist.angular.z])
    corrected_orientation = calculate_quaternion_update(current_orientation, current_angular_velocity, dt)

    #create the pose_stamped msg
    corrected_odom = odometry_msg
    corrected_odom.header.stamp = odometry_msg.header.stamp +rospy.Duration.from_sec(dt) 
    corrected_odom.pose.pose.position.x = corrected_position[0] 

    corrected_odom.pose.pose.position.y = corrected_position[1]  
    corrected_odom.pose.pose.position.z = corrected_position[2] 

    # Set the orientation of the pose using the quaternion

    corrected_odom.pose.pose.orientation.x = corrected_orientation[0]
    corrected_odom.pose.pose.orientation.y = corrected_orientation[1]
    corrected_odom.pose.pose.orientation.z = corrected_orientation[2]
    corrected_odom.pose.pose.orientation.w = corrected_orientation[3]
    return corrected_odom


def create_transform_from_odometry(odometry_msg):
    transform = TransformStamped()
    transform.header = odometry_msg.header
    transform.child_frame_id = odometry_msg.child_frame_id
    transform.transform.translation = odometry_msg.pose.pose.position
    transform.transform.rotation = odometry_msg.pose.pose.orientation
    return transform

def Filtered_odometry(state,cov,header,visu):
    Psn = Odom2DWithCovariance()
    Psn.header = header
    Psn.pose.x = state[0]
    Psn.pose.y  = state[1]
    Psn.pose.theta = state[3]
    Psn.velocity = state[2]
    Psn.angular_velocity =state[4]
    covar = cov.flatten().tolist()
    Psn.covariance = covar
    Psn.Visu = visu
    return Psn 


def Filtered_zero(Header):
    # Function used to create an empty message so that the topic of the filtered odometry is used 
    # Even if the filter is not initialized yet because the target has not been seen yet.
    empty_msg = Odom2DWithCovariance()
    # the good header is used to have the right time stamp
    # all the other values are 0 except velocity that is 1000 to say that the filter is not initialized yet.
    #  and the Visu variable is False to say that no target is visible
    empty_msg.header = Header
    empty_msg.pose.x = 0
    empty_msg.pose.y  = 0
    empty_msg.pose.theta = 0
    empty_msg.velocity = 1000
    empty_msg.angular_velocity = 0
    empty_msg.Visu = False
    false_cov = np.zeros((5,5))
    empty_msg.covariance = false_cov.flatten().tolist()
    return empty_msg


def Unfiltered_state(Header,X,Y,heading):
    # Create a message to publish the unfiltered state so the velocities are 0 because not evaluated
    # and the covariace is O because there is no filtering operation
    Unfiltered_position = Odom2DWithCovariance()
    Unfiltered_position.header = Header
    Unfiltered_position.pose.x = X
    Unfiltered_position.pose.y  = Y
    Unfiltered_position.pose.theta = heading
    Unfiltered_position.velocity = 0
    Unfiltered_position.angular_velocity = 0
    Unfiltered_position.Visu = True
    false_cov = np.zeros((5,5))
    Unfiltered_position.covariance = false_cov.flatten().tolist()
    return Unfiltered_position

############################################################
################### UKF functions
############################################################

##### State vector is [X Y Yaw longitudinal_velocity Yaw_rate longitudinal acceleration)]

def iterate_x(x_in, timestep, inputs):
    '''this function is based on the x_dot and can be nonlinear as needed'''
    if x_in[4] == 0:
        x_in[4] = 10**-20
    
    ret = np.zeros(len(x_in))

    ret[0] = x_in[0] + 2*x_in[2]/x_in[4] * np.sin(x_in[4]*timestep/2) * np.cos(x_in[3] + x_in[4]*timestep/2)

    ret[1] = x_in[1] + 2*x_in[2]/x_in[4] * np.sin(x_in[4]*timestep/2) * np.sin(x_in[3] + x_in[4]*timestep/2)

    ret[2] = x_in[2]

    ret[3] = x_in[3] + x_in[4]*timestep

    ret[4] = x_in[4]

    return ret


def get_q_2(timestep,heading,sigma_a,sigma_alpha) :
    sigma = np.array([sigma_a**2,sigma_alpha**2])
    SIGMA = np.diag(sigma)
    #G = np.array([[0,0],[0,0],[0,0],[timestep,0],[0,timestep]])
    h = heading
    G = np.array([[0,0],[0,0],[timestep,0],[0,0],[0,timestep]])

    #G = np.array([[0.5*timestep**2 * math.cos(h),0],[0.5*timestep**2 * math.sin(h),0],[timestep,0],[0,0.5*timestep**2],[0,timestep]])
    Q = G@SIGMA@np.transpose(G)

     

    return Q
     

vax_x = 0.002#0.001998817
var_y = 0.002#0.001998817
var_theta = 0.00022 #0.00021677908

# create measurement noise covariance matrices
r_cam_pos = np.zeros([2, 2])
r_cam_pos[0][0] = vax_x
r_cam_pos[1][1] = var_y


r_cam_h = np.zeros([1, 1])
r_cam_h[0][0] = var_theta

print("Pre-NODE definitions done") #REMOVE

############################################################
################### NODE
############################################################


class MyNode:
    def __init__(self,Name = "Position_calculation"): #TrajX,Trajy,
        rospy.init_node(Name)
        print("Class runs") #REMOVE
        self.Static_transform = self.get_static_transform()
        print("Pre-subscribe done") #REMOVE

        #self.Target_relative_pose = message_filters.Subscriber('/uav/Target_Relative_Pose', PoseStamped)
        self.Target_relative_pose = message_filters.Subscriber('/uav/gimbal/Target_Relative_Pose', PoseStamped)
        print("Post-subscribe done") #REMOVE
        
        self.UAV_odom = message_filters.Subscriber('/uav1/odometry/odom_main', Odometry) #/uav1/ground_truth

        self.ts = message_filters.ApproximateTimeSynchronizer([self.Target_relative_pose, self.UAV_odom], queue_size=10, slop=0.1)
        print("Should be subscribed") #REMOVE

        self.ts.registerCallback(self.callback)
        self.Target_global_pose = rospy.Publisher('/uav/Target_global_pose', Odom2DWithCovariance, queue_size=10) #PoseStamped
        self.Target_global_pose_filtered = rospy.Publisher('/uav/Target_global_pose_filtered', Odom2DWithCovariance, queue_size=10)
        self.UAV_global_pose = rospy.Publisher('/uav/UAV_global_pose', Odometry, queue_size=10)
        self.Previous_measurment = rospy.Time.now().to_sec()
        self.Q = np.empty((0,))


        self.iterator = 0
        self.state_estimator =None
        self.old_theta = 0
        self.turn_number = 0
        self.reaparition = False
        self.Time_out_FOV = 0
        self.corr_quat=np.array([0.0, -0.7071, 0.0, 0.7071])


    def callback(self, PoseStamped_msg,odom_msg):
        ####A message is received  even if the traget is not visible but Z  =1000 and the rest of the variables = 0 if no target is visible
        aquisition_time = PoseStamped_msg.header.stamp
        # Time step since last recieved image calculation.
        Time_step = aquisition_time.to_sec() - self.Previous_measurment
        time_diff = (aquisition_time- odom_msg.header.stamp).to_sec()

        #### Calculate the position of the target.  
        UAV_position = corrected_odometry(odom_msg, time_diff)
        Target_relative_fcu = transform_pose_to_fcu(PoseStamped_msg,self.Static_transform)
        transform_msg = create_transform_from_odometry(UAV_position)
        target_position = transform_to_local_origin(Target_relative_fcu, transform_msg)
        
        self.UAV_global_pose.publish(UAV_position)

        header = target_position.header
        X_pos = target_position.pose.position.x
        Y_pos = target_position.pose.position.y
        Z_pos = target_position.pose.position.z
        orientation_q = target_position.pose.orientation

        Heading = (np.arctan2(2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y), 1 - 2 * (orientation_q.y ** 2 + orientation_q.z ** 2))) + np.pi
        # if Heading < 0: 
        #     Heading = Heading + np.pi

        print("Heading: ",Heading)

        #rospy.loginfo("bite")
        if self.iterator > 0:  
            # Calculate and Unwrap the angle.
            Heading_prime = Heading + self.turn_number * 2 * np.pi
            if Heading_prime - self.old_theta > np.pi: 
                self.turn_number =  self.turn_number- 1
                print("turn number -1")


            elif  Heading_prime - self.old_theta < - np.pi: 
                self.turn_number =  self.turn_number +1
                print("turn number + 1")

        Heading = Heading + self.turn_number * 2 * np.pi
        Heading += np.pi/2


        #If the target is not detected
        if PoseStamped_msg.pose.position.z >500 : 
            #means that no target is detected. 
            rospy.loginfo("No target detected! %s")

            # If the filter is not initialized yet (meaning the target has not been seen yet)
            # Publish an empty message with all 0 except velocity that is 1000 to say that the filter is not initialized yet and Visu = False
            if self.iterator == 0 :
                Empty_fitered_position = Filtered_zero(header)
                self.Target_global_pose_filtered.publish(Empty_fitered_position)

            # If the filter initialized  We publish the prediction ith Visu = False to say that we do not see the target 
            # Initialize a timer to know wen for how long the target is lost
            elif self.iterator > 0 :
                # Initialize a timer to know wen for how long the target is lost
                self.Time_out_FOV += Time_step
                # if the target is not lost for too long we publish the prediction 
                # set the variable reapartion to True that means that the filter will be reset.
                if self.Time_out_FOV < 3: # must be smaller than the minimum time that the target takes do 1/2 of complete turn on itself default 5 seconds
                    #Prediction 
                    state_prev = self.state_estimator.get_state()
                    q = get_q_2(Time_step,state_prev[3],12.31934966,0.08339564)
                    self.Q = q
                    self.state_estimator.predict(Time_step,q)
                    state = self.state_estimator.get_state()
                    cov = self.state_estimator.get_covar()
                    Heading = state[3]
                    self.old_theta = Heading
                    visu = False
                    target_filtered = Filtered_odometry(state,cov,header,visu)
                    self.Target_global_pose_filtered.publish(target_filtered)
                    self.reaparition = True
                # if the target is  lost for too long we publish an empty message 
                # we set the variable reaparition to False be cause the target did n reappear
                # We set the iterator variable to 0 so that the filter will be reinitialized
                else: 
                    Empty_fitered_position = Filtered_zero(header)
                    self.Target_global_pose_filtered.publish(Empty_fitered_position)
                    self.reaparition = False
                    self.iterator = 0


            #measured_state_publisher
            #since the target is not visible fublish an Empty message with Visu = False
            Empty_Unfiltered_position = Filtered_zero(header)
            self.Target_global_pose.publish(Empty_Unfiltered_position)



        else: # The target is detected 
            #initialisation du filtre if necessary 
            if self.iterator == 0:
                # Set the turn number = 0  because it can be != since the initialization can be done if the target 
                # has not been seen sice a long time 
                self.turn_number = 0
                # reinitialization of the time that the target dissipeared
                self.Time_out_FOV = 0
                self.iterator = 1
                initial_state = np.zeros(5)
                initial_state[0] = X_pos
                initial_state[1] = Y_pos
                initial_state[2] = 1
                initial_state[3] = Heading
                initial_state[4] = 0.0
                initial_covar = 5*np.eye(5)
                self.old_theta = Heading
                Initial_process_noise = get_q_2(0.025,Heading,12.31934966,0.08339564)
                self.Q = Initial_process_noise
                self.state_estimator = UKF(5, Initial_process_noise,initial_state, 5*np.eye(5), 0.04, 0.0, 2.0,iterate_x)
                self.state_estimator.reset(initial_state, initial_covar)

            # If the target dissipeared from the FOV but then comes back in the fov not to long after disappearing 
            # Correct the number of turn if necessary.
             
            # !!!!! to be verified because may lead to error sometimes if the angle is near 0 when the target reappears
            elif self.reaparition == True:
                # calculate the difference between the old heading that has been computed 
                self.reaparition = False
                diff_angle = Heading -self.old_theta
                if diff_angle > np.pi/2:
                    self.turn_number -=1
                    Heading -= 2*np.pi
                elif diff_angle < np.pi/2:
                    self.turn_number +=1
                    Heading += 2*np.pi



            self.old_theta = Heading
            cam_mes_pos = np.array([X_pos, Y_pos])
            cam_mes_h = np.array([Heading])
            state_prev = self.state_estimator.get_state()
            q = get_q_2(Time_step,state_prev[3],12.31934966,0.08339564)
            self.Q = q
            self.state_estimator.predict(Time_step,q)
            self.state_estimator.update([0,1], cam_mes_pos,r_cam_pos)
            self.state_estimator.update([3], cam_mes_h, r_cam_h)
            state = self.state_estimator.get_state()
            cov = self.state_estimator.get_covar()
            visu = True

            #Create an odometry message for the filtered pose of the target
            target_filtered = Filtered_odometry(state,cov,header,visu)
            self.Target_global_pose_filtered.publish(target_filtered)

            #measured_state_publisher
            #this message is easier to handle
            target_position_bis = Unfiltered_state(header,X_pos,Y_pos,Heading)
            self.Target_global_pose.publish(target_position_bis)
        
        self.Previous_measurment = aquisition_time.to_sec() 

 
    def get_static_transform(self):
        # Initialize the TF buffer and listener
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        # Wait for the transform from the down_rgbd/color_optical frame to the fcu frame
        while not rospy.is_shutdown():
            print("In loop") #REMOVE
            try:
                transform = tf_buffer.lookup_transform("uav1/fcu", "uav1/down_rgbd/color_optical", rospy.Time(0), rospy.Duration(1.0)) #Leave this because the calculation taking the gimbal angle into account will be done later. This speeds up the program and reduces errors.
                #transform = tf_buffer.lookup_transform("uav1/fcu", "uav1/servo_camera_optical", rospy.Time(0), rospy.Duration(1.0))
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Caught except") #REMOVE
                continue

        #rospy.loginfo("Static transform recorded! %s",transform )

        print("Loop exited") #REMOVE
        return transform



 

if __name__ == '__main__':

    node = MyNode()
    rospy.spin()
