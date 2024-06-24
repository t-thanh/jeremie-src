#!/usr/bin/python3
import rospy
import matplotlib.pyplot as plt
import numpy as np
from math import factorial
from scipy.special import comb

from cvxopt import solvers
from cvxopt import matrix

from numpy.polynomial import Polynomial
from mrs_msgs.msg import Reference
from mrs_msgs.msg import TrajectoryReference

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


import itertools

from tfe_msgs.msg import Odom2DWithCovariance



##########################
##########################

def binomial_coefficient(n, k):
    return comb(n, k, exact=True)

def get_M(degree):
    # Calculates the M array to express a Bézier curve as B(t) = T * M * P 
    # where P is a vector containing the control points an T an array of size t*(degree+1)
    # of the power of the time at which the curve is evaluated
    Bernstein_polyn = np.zeros(degree+1)
    Bezier_mat = np.zeros([degree+1,degree+1])

    for i in range(degree+1):
        Bernstein_polyn[i] = comb(degree, i, exact=True)

    for j in range(degree+1):
        val = Bernstein_polyn[j]
        mat = np.zeros(degree+1)

        for k in range(degree +1 - j):
            if (j % 2) == 0:
                mat[k] = (-1)**((j+1)*(k+1))  *  val * comb(degree-j, k, exact=True)
            else:
                mat[k] =  (-1)**((j)*(k)) * val * comb(degree-j, k, exact=True)
        Bezier_mat[j,:] = mat

    return Bezier_mat


def enumerate_combinations(n, k):
    # create a list of all possible combinations of k elements from the integers 1 to n
    combs = list(itertools.combinations(range(1, n+1), k))

    # convert the list to a NumPy array
    combs = np.array(combs)

    return combs

def get_regularizer(degree):

    # order of the second derivative of a Bézier curve
    derive2_deg = degree-2 

    # get the array to express the second derivative of the bézier curve as a new bézier curve of orde n-2 
    # with new control points that can be calculated based on the control points of the initial Béziier curve
    poly_mat  = np.transpose(get_M(derive2_deg))

    # enumerate the combinaisons of 2 in the numbers of terms of the derivative ( this is to evalute the square of the second derivative of B(t))
    combinaison= enumerate_combinations(derive2_deg+1, 2)

    #initialize the vector that will contain the value of the integral from 0 to 1 of the different terms
    integral = np.zeros(len(combinaison)+derive2_deg+1)

    ## Calculate the value of the ingral of the terms of || B(t)"||^2
    # this loop only calculates for the terms in 2ab eg. it is a summ suqared so for example (a+b+c) = a^2 + b^2 +c^2 +2ab+ 2ac+2bc
    for i in range(len(combinaison)):
        polyn_1 = np.poly1d(np.flip(poly_mat[combinaison[i,0]-1,:]))
        polyn_2 = np.poly1d(np.flip(poly_mat[combinaison[i,1]-1,:]))
        #  I added the () -> 2*() 
        integral[i+derive2_deg+1] = 2*(np.polyval(np.polyint(polyn_1*polyn_2),1) - np.polyval(np.polyint(polyn_1*polyn_2),0))

    # calculates for the squared terms 
    for i in range(derive2_deg+1): 
        polyn =  np.poly1d(np.flip(poly_mat[i,:]))
        integral[i] = np.polyval(np.polyint(polyn*polyn),1) - np.polyval(np.polyint(polyn*polyn),0)

    # Create a diagonal array with the values of the ingrals of the square terms 
    Matrice_prime = np.zeros([derive2_deg+1,derive2_deg+1])
    for i in range(derive2_deg+1):
        Matrice_prime[i,i] = integral[i]

    # Puts the othr elements in the array (the arry is symmetric)
    for i in range(len(combinaison)):
             Matrice_prime[combinaison[i,0]-1,combinaison[i,1]-1] = integral[i+derive2_deg+1]/2
             Matrice_prime[combinaison[i,1]-1,combinaison[i,0]-1] = integral[i+derive2_deg+1]/2
    # multiply by n^2 * (n-1)^2
    Matrice_prime = Matrice_prime*(degree)**2*(degree-1)**2
    
    # Initializes the matrix to go from the control poits of B(t)" to the control points of B(t)
    Mat_passage = np.zeros([derive2_deg +1,degree+1])
    for i in range(derive2_deg +1):
        Mat_passage[i,i] = 1
        Mat_passage[i,i+1] = -2
        Mat_passage[i,i+2] = 1
    Mat = np.transpose(Mat_passage)@Matrice_prime@Mat_passage
    return Mat    

def get_velocity(P):
    
    Mat_passage = np.zeros((P.shape[1]-1, P.shape[1]))
    np.fill_diagonal(Mat_passage, -1)
    np.fill_diagonal(Mat_passage[:,1:], 1)
    P_prime = (P.shape[1]-1)*(Mat_passage @ np.transpose(P))
    return P_prime


def get_acceleration(degree,P):

    vel = get_velocity(P)
    acc = get_velocity(vel)
    return acc

def get_Max(P):
    M = get_M(len(P))
    polyn_x = np.poly1d(np.flip(M@P[0,:]))
    polyn_y = np.poly1d(np.flip(M@P[1,:]))
    squared_norm = np.polypow(polyn_x,2) + np.polypow(polyn_y,2)
    Extrema = np.polyroot(np.polyder(squared_norm))

    return Extrema

def check_constraints(constraint, control_points,time_scale):
    M = get_M(len(control_points)-1)
    polyn_x = np.poly1d((M@control_points[:,0]))
    polyn_y = np.poly1d((M@control_points[:,1]))
    squared_norm = (polyn_x*polyn_x) + (polyn_y*polyn_y)
    

    constr = np.poly1d(np.array([constraint**2]))
    zeros = np.roots(squared_norm/time_scale - constr)
    real_valued = zeros.real[abs(zeros.imag)<1e-5] # where I chose 1-e5 as a threshold
    for i in real_valued: 
        if i >= 0 and i <=1: 
            #print("non conforme")
            #print(real_valued)
            return False
    
    #print("conforme")
    #print(real_valued)
    return True
    
           
            



    


##################


class Queue:
    def __init__(self, Max_len, dim):
        self.L = Max_len
        self.Q = np.zeros([1, dim])
        self.Time = np.zeros(1)
        self.Weight = np.empty(0)
        self.Time_scaled = np.empty(0)
        self.Index = 0

    def push_element(self, element, T):
        if self.Index >=  1:
            self.Q = np.vstack([self.Q,element])
            self.Time = np.append(self.Time,T)

            if self.Q.shape[0] > self.L:
                self.Q = np.delete(self.Q, 0, axis=0)
                self.Time = np.delete(self.Time, 0)
        else :
            if self.Index == 0: 
                self.Q[0] = element 
                self.Time[0] = T
                self.Index +=1



    def Calculate_weight(self,Factor,max_time):
        self.Weight =  np.zeros(len(self.Time))
        current_length = max(self.Q.shape)
        non_zero_length = max(self.Q.shape)
        for i in range(current_length):
            if i <  current_length-1:#(self.L -1):
                if (self.Time[-1]-self.Time[i]) > max_time:
                    self.Weight[i] = 0
                    non_zero_length = current_length - (i +1)

                else:
                    self.Weight[i] = np.tanh(Factor/(self.Time[-1]-self.Time[i]))

            else:
                self.Weight[-1] = 1
        rospy.loginfo("non_zero_length: {}".format(non_zero_length))
        return non_zero_length



    def scale_time(self,factor):
        self.Time_scaled = np.zeros(len(self.Time))
        self.Time_scaled = (self.Time - self.Time[0])/(self.Time[-1]-self.Time[0])*factor


def get_T(Time_scaled,degree): 

    L = len(Time_scaled)
    T = np.zeros([L,degree + 1])

    for i in range(degree+1): 
        if i < degree: 
            T[:,i] = Time_scaled**(degree-i)
        else:
            T[:,degree] = np.ones(L)

    return T

def get_W(Weight):
    W = np.diag(Weight)
    return W


def get_G(degree):
    A = np.zeros([2*degree-1,degree +1])
    for i in range(degree):    
        A[i,i] = -1*degree
        A[i,i+1] = 1*degree
    for i in range(degree-1):
        A[i+degree,i] = degree*(degree-1)*1
        A[i+degree,i+1] = -2*degree*(degree-1)
        A[i+degree,i+2] = degree*(degree-1)*1

    G = np.concatenate((A,-A),axis = 0)
    return G



def get_h(degree,v_max,a_max):
    u = np.zeros(2*(2*degree-1))
    u[0:degree] = v_max
    u[degree:degree+2] = a_max
    u[degree+2:2*degree+2] = v_max
    u[2*degree +2:] =  a_max
    return u



##########################

def Reference_message_creator(Bezier_control_points,M,aquisition_time,temps_queue,Pred_Len = 2.5,N_points = 50):
        Traj = TrajectoryReference()
        Traj.header.frame_id = ""
        Traj.header.stamp    = rospy.Time.now()
        Traj.fly_now     = True
        Traj.use_heading = False
        Delta_T = rospy.Time.now().to_sec() - aquisition_time.to_sec()
        Traj_Len =Pred_Len - Delta_T
        dt = Traj_Len/N_points 
        Traj.dt = dt

        factor_bis = (temps_queue+Delta_T)/(temps_queue+Pred_Len)
        sampling_instants= np.linspace(factor_bis,1,N_points+1)

        T2 = get_T(sampling_instants,max(Bezier_control_points.shape)-1) # len(self.Regulirizer)-1 is the degree of the bézier curve
                
        curvex = T2@M@Bezier_control_points[0,:]
        curvey = T2@M@Bezier_control_points[1,:]

        for i in range(N_points +1):
            Ref = Reference()
            Ref.position.x = curvex[i]
            Ref.position.y = curvey[i]
            Ref.position.z = 0
            
            Ref.heading = 0.0
            Traj.points.append(Ref)

        return Traj


def Path_message_creator(Bezier_control_points,M,aquisition_time,temps_queue,Pred_Len = 2.5,N_points = 25):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        Delta_T = rospy.Time.now().to_sec() - aquisition_time.to_sec()
        Traj_Len =Pred_Len - Delta_T
        dt = Traj_Len/N_points 

        factor_bis = (temps_queue+Delta_T)/(temps_queue+Pred_Len)
        sampling_instants= np.linspace(factor_bis,1,N_points)

        T2 = get_T(sampling_instants,max(Bezier_control_points.shape)-1) # len(self.Regulirizer)-1 is the degree of the bézier curve
                
        curvex = T2@M@Bezier_control_points[0,:]
        curvey = T2@M@Bezier_control_points[1,:]

        for i in range(N_points):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map' # Change this to the frame id of your map
            pose_stamped.pose.position.x = curvex[i]
            pose_stamped.pose.position.y = curvey[i]
            pose_stamped.pose.position.z = 0.1
            path_msg.poses.append(pose_stamped)


        return path_msg


##########################
#rospy.core.ROSNode
class MyNode:
    def __init__(self,deg,Visualisation_ = False,Name = "Bezier_predict"): #TrajX,Trajy,
        rospy.init_node(Name)
        self.Odom_sub2 = rospy.Subscriber('/uav/Target_global_pose_corrected',Odom2DWithCovariance,self.callback)
        #self.Odom_sub2 = rospy.Subscriber('/uav/Target_global_pose_filtered',Odom2DWithCovariance,self.callback)
        self.Target_traj = rospy.Publisher('/uav/Target_Bezier_Traj',TrajectoryReference, queue_size=1)
        self.path_pub = rospy.Publisher('/Target/path', Path, queue_size=10)
        self.queue = Queue(50,2)
        self.QL = 50
        self.Tw = 0.01
        self.degree = deg
        self.Regulirizer = get_regularizer(deg)
        self.M = get_M(deg)
        self.Flag = False
        self.Solution = np.empty(0)
        self.temps_pred = 2.5 
        self.temps_queue = 0
        self.Visualisation =  Visualisation_
        self.G = get_G(deg)
        if Visualisation_ == True:
            rospy.loginfo("Visu = true2")

            self.path_pub = rospy.Publisher('/Target/path', Path, queue_size=10)


    def callback(self,Target_psn):
        print("Callback")
        if Target_psn.Visu == True:
            rospy.loginfo("Target in FOV")
            aquisition_time = Target_psn.header.stamp

            X = Target_psn.pose.x
            Y = Target_psn.pose.y 

            Pos = np.array([X,Y])
            self.queue.push_element(Pos,aquisition_time.to_sec())
            not_too_old_elem = self.queue.Calculate_weight(0.5,3) 
        
            if len(self.queue.Time) < 7:
                rospy.loginfo("Less than 7 position recorded")
                self.Flag = False

            elif not_too_old_elem < 7:

                rospy.loginfo("Less than 7 non zero position recorded")
                self.Flag = False


            else: 
                max_vel = 10
                max_acc = 5
                self.temps_queue = (self.queue.Time[-1] - self.queue.Time[0])
                time_scale = (self.temps_pred+ self.temps_queue)
                factor = self.temps_queue/(self.temps_pred+ self.temps_queue)
                self.Bezier_fitting(0.25,factor,time_scale*max_vel,time_scale*max_acc,False)

                vel = get_velocity(self.Solution)
                acc = get_velocity(vel)
                if not (check_constraints(max_vel, vel,time_scale) and check_constraints(max_acc, acc,time_scale)):
	
                    #self.Solution=self.Solution = self.Bezier_fitting(0.25,factor,time_scale*max_vel,time_scale*max_acc,True)
                    self.Solution = self.Bezier_fitting(0.25,factor,time_scale*max_vel,time_scale*max_acc,True)
                    rospy.loginfo("Constraint application")
                
                    print("Flag =",self.Flag)
                if self.Flag == True:
                    Future_Trajecotry = Reference_message_creator(self.Solution,self.M,aquisition_time,self.temps_queue,Pred_Len = 2.5,N_points = 50)
                    rospy.loginfo("Traj_published")
                    self.Target_traj.publish(Future_Trajecotry)
                    
                    if self.Visualisation == True:
                        Path =  Path_message_creator(self.Solution,self.M,aquisition_time,self.temps_queue,Pred_Len = 2.5,N_points = 25)
                        self.path_pub.publish(Path)

                else:
                    rospy.loginfo("Error older solution kept" )
        else : 
            rospy.loginfo("Target out FOV No trajectory calculated ")

 
        

    def Bezier_fitting(self,weight_factor,factor,max_vel,max_acc,constraint_flag = False):
        self.queue.scale_time(factor)
        #self.queue.Calculate_weight(weight_factor)

        T = get_T(self.queue.Time_scaled,self.degree)
        W = get_W(self.queue.Weight)
        X = self.queue.Q[:,0]
        Y = self.queue.Q[:,1]

        if constraint_flag == True:
            G = matrix(self.G)
            h = matrix(get_h(self.degree,max_vel,max_acc))
            print("real constrain application")
        else:
            G = None
            h = None


        # P1 = ((np.transpose(T@M)@(W@(T@M))))
        qx = ((np.transpose((T@self.M))@(W@X)))
        qy = ((np.transpose((T@self.M))@(W@Y)))
        P2 = self.Regulirizer

        P = ((np.transpose(T@self.M)@(W@(T@self.M)))+self.QL*self.Tw*P2)
        A = None
        b = None

        # Solve the problem
        try : 

            solx = solvers.qp(matrix(P), matrix(-qx), G, h, A, b, solver='OSQP',verbose=False)
            soly= solvers.qp(matrix(P), matrix(-qy), G, h, A, b,solver='OSQP',verbose=False)

            solX = np.squeeze(solx['x']) 
            solY = np.squeeze(soly['x'])
            self.Solution = np.vstack([solX,solY])
            print("Solution Found" )
            self.Flag = True
            return self.Solution
            
        except:
            print("Error older solution kept" )
            self.Flag = False





if __name__ == '__main__':
    #rospy.init_node('Bézier_predict')
    node = MyNode(5,True)
    rospy.spin()
