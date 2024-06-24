#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <sstream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <string>

#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/String.h"
#include <std_srvs/Trigger.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

#include <message_filters/synchronizer.h>

// for storing information about the state of the uav (position)
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
// for storing information about the state of the uav (position, twist) + covariances
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

// custom msgs of MRS group //
#include <mrs_lib/attitude_converter.h>
#include <mrs_msgs/UavState.h>
#include <mrs_msgs/Reference.h>
#include <mrs_msgs/TrajectoryReference.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/ReferenceStamped.h>

#include <mrs_msgs/TrajectoryReferenceSrv.h>
#include <mrs_msgs/TrajectoryReferenceSrvRequest.h>
#include <mrs_msgs/TrajectoryReferenceSrvResponse.h>

// Specific header libraries
#include <traj_min_jerk.hpp>
#include "lbfgs.hpp"
#include <chrono>
#include <gdcpp.h>
#include <Corridor_generation.h>
#include <cost_function.h>
#include <constraint_verification.h>
////////////////
#include "osqp.h"
#include "OsqpEigen/OsqpEigen.h"


////////////////
// #include "matplotlibcpp.h"
// namespace plt = matplotlibcpp;

//////////////////////////////////
// Minimization function
//////////////////////////////////
class Cost_fun_Minimizer
{
public:
    std::vector<Eigen::MatrixXd *> A;
    Eigen::MatrixXd b;
    int M;
    CF::Cost_function Cost_fun;

    Cost_fun_Minimizer(Eigen::Matrix3d iS, Eigen::Matrix3d fS, int M, std::vector<Eigen::MatrixXd *> Left_mat, Eigen::MatrixXd right_mat, Eigen::MatrixXd min_time, const Eigen::VectorXd &K, const Eigen::VectorXd &rho_v, const Eigen::VectorXd &rho_a)
    {
        A = Left_mat;
        b = right_mat;
        Cost_fun.initialize(M, iS, fS, min_time, K, rho_v, rho_a);
    }
    Eigen::VectorXd run(const Eigen::VectorXd &X)
    {
        double finalCost;
        Eigen::VectorXd x = X;
        /* Set the minimization parameters */
        lbfgs::lbfgs_parameter_t params;
        params.g_epsilon = 1; // 1.0e-2;
        params.past = 8;
        params.delta = 1.0e-2;
        params.max_linesearch = 64;

        /* Start minimization */
        int ret = lbfgs::lbfgs_optimize(x,
                                        finalCost,
                                        costFunction,
                                        nullptr,
                                        nullptr, // monitorProgress,//monitorProgress,
                                        this,
                                        params);

        /* Report the result. */
        // std::cout << "================================" << std::endl;
        // if (ret < 0) {
        //   std::cout << "L-BFGS Optimization Failed: " << lbfgs::lbfgs_strerror(ret) << std::endl;
        // }
        // std::cout << "L-BFGS Optimization Returned: " << ret << std::endl
        //   << "Minimized Cost: " << finalCost << std::endl;
        //   << "Optimal Variables: " << std::endl
        //   << x.transpose() << std::endl;

        // return ret;
        return x;
    }

private:
    static double costFunction(void *instance,
                               const Eigen::VectorXd &x,
                               Eigen::VectorXd &g)
    {
        Cost_fun_Minimizer *example = reinterpret_cast<Cost_fun_Minimizer *>(instance);
        double fx = 0.0;
        double Cost = example->Cost_fun.variable_points_update(x, example->A, example->b, 8, 4);
        g = example->Cost_fun.Full_grad;
        fx += Cost;

        return fx;
    }

    static int monitorProgress(void *instance,
                               const Eigen::VectorXd &x,
                               const Eigen::VectorXd &g,
                               const double fx,
                               const double step,
                               const int k,
                               const int ls)
    {
        std::cout << "================================" << std::endl
                  << "Iteration: " << k << std::endl
                  << "Function Value: " << fx << std::endl
                  << "Gradient Inf Norm: " << g.cwiseAbs().maxCoeff() << std::endl
                  << "Variables: " << std::endl
                  << x.transpose() << std::endl;
        return 0;
    }
};

////////////////////////
///
////////////////////////

bool all_positive(const Eigen::VectorXd &v)
{
    for (int i = 0; i < v.size(); ++i)
    {
        if (v(i) <= 0)
        {
            return false;
        }
    }
    return true;
}

///////////////////////////////////
////// Optimize Heading
//////////////////////////////////

class Heading_optimizer
{
public:
    double degree = 8;

    Eigen::MatrixXd M;
    Eigen::MatrixXd P2;
    double omega_max;
    double omega_point_max;
    Eigen::MatrixXd A;
    Eigen::SparseMatrix<double> sparse_A;
    Eigen::VectorXd l;
    Eigen::VectorXd u;
    int order = 8;
    Eigen::VectorXi index;
    Eigen::VectorXd Time_vector;
    Eigen::MatrixXd T;
    Eigen::MatrixXd W;
    Eigen::MatrixXd P1;
    Eigen::SparseMatrix<double> P;

    Heading_optimizer()
    {
        omega_max = 0.8;
        omega_point_max = 0.8;
        M.setZero(9, 9);
        M << -1.0, 8.0, -28.0, 56.0, -70.0, 56.0, -28.0, 8.0, -1.0,
            8.0, -56.0, 168.0, -280.0, 280.0, -168.0, 56.0, -8.0, 0.0,
            -28.0, 168.0, -420.0, 560.0, -420.0, 168.0, -28.0, 0.0, 0.0,
            56.0, -280.0, 560.0, -560.0, 280.0, -56.0, 0.0, 0.0, 0.0,
            -70.0, 280.0, -420.0, 280.0, -70.0, 0.0, 0.0, 0.0, 0.0,
            56.0, -168.0, 168.0, -56.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            -28.0, 56.0, -28.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            8.0, -8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        P2.setZero(9, 9);

        P2 << 2.41230769e+02, -3.61846154e+02, 5.48251748e+01, 3.28951049e+01, 1.82750583e+01, 9.13752914e+00, 3.91608392e+00, 1.30536131e+00, 2.61072261e-01,
            -3.61846154e+02, 6.14041958e+02, -1.53510490e+02, -7.31002331e+01, -2.74125874e+01, -5.22144522e+00, 2.61072261e+00, 3.13286713e+00, 1.30536131e+00,
            5.48251748e+01, -1.53510490e+02, 1.09650350e+02, 1.82750583e+01, -1.43589744e+01, -1.56643357e+01, -5.74358974e+00, 2.61072261e+00, 3.91608392e+00,
            3.28951049e+01, -7.31002331e+01, 1.82750583e+01, 3.13286713e+01, 1.17482517e+01, -9.39860140e+00, -1.56643357e+01, -5.22144522e+00, 9.13752914e+00,
            1.82750583e+01, -2.74125874e+01, -1.43589744e+01, 1.17482517e+01, 2.34965035e+01, 1.17482517e+01, -1.43589744e+01, -2.74125874e+01, 1.82750583e+01,
            9.13752914e+00, -5.22144522e+00, -1.56643357e+01, -9.39860140e+00, 1.17482517e+01, 3.13286713e+01, 1.82750583e+01, -7.31002331e+01, 3.28951049e+01,
            3.91608392e+00, 2.61072261e+00, -5.74358974e+00, -1.56643357e+01, -1.43589744e+01, 1.82750583e+01, 1.09650350e+02, -1.53510490e+02, 5.48251748e+01,
            1.30536131e+00, 3.13286713e+00, 2.61072261e+00, -5.22144522e+00, -2.74125874e+01, -7.31002331e+01, -1.53510490e+02, 6.14041958e+02, -3.61846154e+02,
            2.61072261e-01, 1.30536131e+00, 3.91608392e+00, 9.13752914e+00, 1.82750583e+01, 3.28951049e+01, 5.48251748e+01, -3.61846154e+02, 2.41230769e+02;

        A.setZero(18, 9);
        A << 1, 0, 0, 0, 0, 0, 0, 0, 0,
            -9, 9, 0, 0, 0, 0, 0, 0, 0,
            56, -112, 56, 0, 0, 0, 0, 0, 0,
            0, -9, 9, 0, 0, 0, 0, 0, 0,
            0, 0, -9, 9, 0, 0, 0, 0, 0,
            0, 0, 0, -9, 9, 0, 0, 0, 0,
            0, 0, 0, 0, -9, 9, 0, 0, 0,
            0, 0, 0, 0, 0, -9, 9, 0, 0,
            0, 0, 0, 0, 0, 0, -9, 9, 0,
            0, 0, 0, 0, 0, 0, 0, -9, 9,
            0, 56, -112, 56, 0, 0, 0, 0, 0,
            0, 0, 56, -112, 56, 0, 0, 0, 0,
            0, 0, 0, 56, -112, 56, 0, 0, 0,
            0, 0, 0, 0, 56, -112, 56, 0, 0,
            0, 0, 0, 0, 0, 56, -112, 56, 0,
            0, 0, 0, 0, 0, 0, 56, -112, 56,
            0, 0, 0, 0, 0, 0, 0, 0, 1,
            0, 0, 0, 0, 0, 0, 0, -9, 9;

        sparse_A.resize(A.rows(), A.cols());
        for (int i = 0; i < A.rows(); i++)
        {
            for (int j = 0; j < A.cols(); j++)
            {
                if (A(i, j) != 0)
                {
                    sparse_A.insert(i, j) = A(i, j);
                }
            }
        }
        u.setZero(18);
        l.setZero(18);
        u << 0, 0, 0, omega_max, omega_max, omega_max, omega_max, omega_max, omega_max, omega_max, omega_point_max, omega_point_max, omega_point_max, omega_point_max, omega_point_max, omega_point_max, 0, 0;
        l = -u;
        W = Eigen::MatrixXd::Identity(9, 9);
    }

    void Optimize(Eigen::Vector3d Heading_state, Eigen::VectorXd Target_traj_x, Eigen::VectorXd Target_traj_y, mrs_msgs::TrajectoryReference &UAV_traj)
    {
        int Point_number = UAV_traj.points.size();
        double dt = UAV_traj.dt;
        double traj_time = dt * Point_number;

        l = l * traj_time;
        u = u * traj_time;

        l(0) = Heading_state(0) - 0.0001;
        l(1) = Heading_state(1) * traj_time - 0.0001;
        l(2) = Heading_state(2) * traj_time - 0.0001;
        u(0) = Heading_state(0) + 0.0001;
        u(1) = Heading_state(1) * traj_time + 0.0001;
        u(2) = Heading_state(2) * traj_time + 0.0001;

        if (Point_number < 9 + 5)
        {
            ROS_INFO("Not enough points to optimize yaw");
            return;
        }
        else
        {

            Eigen::VectorXd Target_angle(9);

            index = Eigen::VectorXi::LinSpaced(9, 5, Point_number);

            Time_vector = (index.cast<double>() * dt) / traj_time;

            T.resize(Time_vector.size(), order + 1);

            for (int i = 0; i < Time_vector.size(); i++)
            {
                for (int j = 0; j < order + 1; j++)
                {
                    T(i, j) = pow(Time_vector(i), order - j);
                }
            }

            P1 = (T * M).transpose() * W * T * M;

            P.resize(9, 9);
            for (int i = 0; i < 9; i++)
            {
                for (int j = 0; j <= i; j++)
                {
                    P.insert(i, j) = P1(i, j) + 0.5 * P2(i, j);
                    if (i != j)
                    {
                        P.insert(j, i) = P1(i, j) + 0.5 * P2(i, j);
                    }
                }
            }

            Eigen::VectorXi index_bis = Eigen::VectorXi::LinSpaced(9, 10, 50);

            for (int i = 0; i < 9; i++)
            {

                double v_x = Target_traj_x(index_bis(i)) - Target_traj_x(index_bis(i) - 1);
                double v_y = Target_traj_y(index_bis(i)) - Target_traj_y(index_bis(i) - 1);
                Target_angle(i) = -atan2(v_x, v_y);
            }

            l(16) = Heading_state(0) - 1.57;
            u(16) = Heading_state(0) + 1.57;
            u(17) = +0.05;
            l(17) = -0.05;

            Eigen::VectorXd Q1;

            Q1 = (T * M).transpose() * W * Target_angle;

            OsqpEigen::Solver solver;
            solver.data()->setNumberOfVariables(9);
            solver.data()->setNumberOfConstraints(18);
            solver.data()->setHessianMatrix(P);
            solver.data()->setGradient(Q1);
            solver.data()->setLinearConstraintsMatrix(sparse_A);
            solver.data()->setLowerBound(l);
            solver.data()->setUpperBound(u);

            // Solve problem
            solver.initSolver();
            solver.solveProblem();

            // Get solution
            Eigen::VectorXd x = solver.getSolution();

            // std::cout << "Solution:\n"
            //         << x << std::endl;
            // std::cout << "Target angle:\n"
            //         << Target_angle <<std::endl;
            UAV_traj.use_heading = true;

            Eigen::VectorXd time2 = Eigen::VectorXd::LinSpaced(Point_number, 0, 1);

            Eigen::MatrixXd T2(Point_number, 9);

            for (int i = 0; i < Point_number; i++)
            {
                for (int j = 0; j < 9; j++)
                {
                    T2(i, j) = pow(time2(i), (order - j));
                }
            }

            Eigen::VectorXd SOL = T2 * -M * x;
            // std::cout<< "SOL" << std::endl;
            // std::cout << SOL << std::endl;
            for (int i = 0; i < Point_number; i++)
            {
                UAV_traj.points[i].heading = SOL(i);
            }
        }
    }
};

///////////////////////////////////
/// ROS
//////////////////////////////////

void get_trajectory_coordinates(const mrs_msgs::TrajectoryReferenceConstPtr &trajectory, Eigen::VectorXd &x_coords, Eigen::VectorXd &y_coords)
{
    // Get the number of trajectory points
    int num_points = trajectory->points.size();

    // Resize the coordinate vectors
    x_coords.resize(num_points);
    y_coords.resize(num_points);

    // Fill in the coordinate vectors
    for (int i = 0; i < num_points; i++)
    {
        x_coords(i) = trajectory->points[i].position.x;
        y_coords(i) = trajectory->points[i].position.y;
    }
}

geometry_msgs::PointStamped odomToPointStamped(const nav_msgs::Odometry &odom_msg, const std::string &frame_id = "map")
{
    geometry_msgs::PointStamped point_stamped;
    point_stamped.header = odom_msg.header;
    point_stamped.header.frame_id = frame_id;
    point_stamped.point.x = odom_msg.pose.pose.position.x;
    point_stamped.point.y = odom_msg.pose.pose.position.y;
    point_stamped.point.z = odom_msg.pose.pose.position.z;
    return point_stamped;
}

nav_msgs::Path UAV_Path(min_jerk::Trajectory minJerkTraj, double Traj_time, double Point_number = 25)
{
    nav_msgs::Path UAV_path;
    UAV_path.header.frame_id = "map";
    Eigen::VectorXd sample = Eigen::VectorXd::LinSpaced(Point_number, 0, Traj_time);

    for (int i = 0; i < Point_number; i++)
    {
        geometry_msgs::PoseStamped point;

        Eigen::Vector3d pos = minJerkTraj.getPos(sample(i));
        point.header.frame_id = "map";
        point.pose.position.x = pos(0);
        point.pose.position.y = pos(1);
        point.pose.position.z = 0.2;
        UAV_path.poses.push_back(point);
    }
    return UAV_path;
}

mrs_msgs::TrajectoryReference UAV_TrajectoryReference_generatation(const ros::Time &Time_init, double Traj_time_corrected, min_jerk::Trajectory minJerkTraj, double altitude, double dt, int Point_number, double heading_init)
{
    ROS_INFO("Appel de UAV_TrajectoryReference_generatation ");
    mrs_msgs::TrajectoryReference Traj_ref;
    Traj_ref.header.stamp = Time_init;
    Traj_ref.header.frame_id = "";
    Traj_ref.fly_now = true;
    Traj_ref.dt = dt;

    // attention régler trjectoire négative

    Traj_ref.use_heading = false;
    Eigen::VectorXd sample = Eigen::VectorXd::LinSpaced(Point_number + 1, 0, Traj_time_corrected);

    for (int i = 0; i < Point_number; i++)
    {
        mrs_msgs::Reference waypoint;

        Eigen::Vector3d pos = minJerkTraj.getPos(sample(i));

        waypoint.position.x = pos(0);
        waypoint.position.y = pos(1);
        waypoint.position.z = altitude;
        waypoint.heading = heading_init;
        Traj_ref.points.push_back(waypoint);
    }

    return Traj_ref;
}

////////////////
///////////////
visualization_msgs::Marker createPolygonMarker(const Eigen::VectorXd& x, const Eigen::VectorXd& y,int id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 0.01;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.id = id;

    for (int i = 0; i < x.size() - 2; ++i)
    {
        geometry_msgs::Point point1;
        point1.x = x(0);
        point1.y = y(0);
        point1.z = 0.0;
        marker.points.push_back(point1);

        geometry_msgs::Point point2;
        point2.x = x(i + 1);
        point2.y = y(i + 1);
        point2.z = 0.0;
        marker.points.push_back(point2);

        geometry_msgs::Point point3;
        point3.x = x(i + 2);
        point3.y = y(i + 2);
        point3.z = 0.0;
        marker.points.push_back(point3);
    }

    return marker;
}


void removeAllMarkers(ros::Publisher& marker_pub)
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(marker);
    marker_pub.publish(marker_array);
}

///////////////
///////////////

///////////////////////////////////
/// Node
//////////////////////////////////

class UAV_trajectory_optimizer_node
{
public:
    ros::NodeHandle nh_;
    message_filters::Subscriber<mrs_msgs::TrajectoryReference> Target_trajectory;
    message_filters::Subscriber<mrs_msgs::UavState> uav_odometry_subscriber;
    ros::Publisher UAV_traj_rviz;
    ros::Publisher UAV_corridor_rviz;

    ros::ServiceServer service_server_activate_;

    ros::ServiceClient service_client_trajectory_;
    typedef message_filters::sync_policies::ApproximateTime<mrs_msgs::TrajectoryReference, mrs_msgs::UavState> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;
    Heading_optimizer Heading_calc;
    double dt = 0.1;
    float requested_alt; // Added member variable, REMOVE if error

    // Constructor
    UAV_trajectory_optimizer_node(float& arg_value) : requested_alt(arg_value)
    {
        Target_trajectory.subscribe(nh_, "/uav/Target_Bezier_Traj", 10);
        uav_odometry_subscriber.subscribe(nh_, "/uav1/odometry/uav_state", 10);
        sync_.reset(new Sync(MySyncPolicy(10), Target_trajectory, uav_odometry_subscriber));
        sync_->registerCallback(boost::bind(&UAV_trajectory_optimizer_node::UAV_traj_optimizer_calleback, this, _1, _2));

        UAV_traj_rviz = nh_.advertise<nav_msgs::Path>("UAV/trajectory", 1);
        UAV_corridor_rviz = nh_.advertise<visualization_msgs::MarkerArray>("UAV/flight_corridor", 1);

        service_server_activate_ = nh_.advertiseService("activate_in", &UAV_trajectory_optimizer_node::callbackActivate, this);
        service_client_trajectory_ = nh_.serviceClient<mrs_msgs::TrajectoryReferenceSrv>("/uav1/control_manager/trajectory_reference");
        Heading_calc = Heading_optimizer();
    }

    void UAV_traj_optimizer_calleback(const mrs_msgs::TrajectoryReferenceConstPtr &Target_traj, const mrs_msgs::UavStateConstPtr &Uav_odom)
    {
        ROS_INFO("Beginning Optimization process");

        ros::Time time_init = ros::Time::now(); // get current time
        bool use_heading = false;
        int Nbr_verification = 10; //(Number of verifaction points per sergement in constrain verification)
        double altitude = requested_alt;
        double max_vel = 8;
        double max_acc = 4;
        double distance_treshold = 1; // Distance treshold for the resampling
        double Max_Time_sample = 0.5; // Trajectory sampling perieod
        double Max_dist = 4;
        int Max_points = 6;
        int Nbr_inital_sample = Target_traj->points.size();
        double FOV_dist = 5;
        double dt = Target_traj->dt;
        bool visualisation = true;
        int Point_number;

        ////////////////////////////

        ros::Time Time_init = ros::Time::now();
        Eigen::VectorXd Target_traj_x;
        Eigen::VectorXd Target_traj_y;
        get_trajectory_coordinates(Target_traj, Target_traj_x, Target_traj_y);

        corridor_generation::Corridor Visible_region = corridor_generation::Corridor(Nbr_inital_sample, Max_Time_sample, dt, Target_traj_x, Target_traj_y, distance_treshold, Max_dist, Max_points, FOV_dist);
        int Number_of_points = Visible_region.M;

        ///////////////////////////////////
        //// Initial and Final position
        //////////////////////////////////
        // Set the initial point to the current uav odometry
        double UAV_px = Uav_odom->pose.position.x;
        double UAV_py = Uav_odom->pose.position.y;
        double UAV_vx = Uav_odom->velocity.linear.x;
        double UAV_vy = Uav_odom->velocity.linear.y;
        double UAV_ax = Uav_odom->acceleration.linear.x;
        double UAV_ay = Uav_odom->acceleration.linear.y;

        Eigen::Matrix3d iS, fS;
        iS.setZero();
        iS(0, 0) = UAV_px;
        iS(1, 0) = UAV_py;
        iS(0, 1) = UAV_vx;
        iS(1, 1) = UAV_vy;
        iS(0, 2) = UAV_ax;
        iS(1, 2) = UAV_ay;

        // Set the last point to the las point of the traj
        // vel and acceleration are calculated by finite difference
        int last_index = Visible_region.Index_vec(Number_of_points);
        double target_vend_x = (Target_traj_x(last_index) - Target_traj_x(last_index - 1)) / dt;
        double target_vend_y = (Target_traj_y(last_index) - Target_traj_y(last_index - 1)) / dt;
        double velocity = sqrt(target_vend_x * target_vend_x + target_vend_y * target_vend_y);
        double targe_aend_x = (target_vend_x - (Target_traj_x(last_index - 1) - Target_traj_x(last_index - 2)) / dt) / dt;
        double targe_aend_y = (target_vend_y - (Target_traj_y(last_index - 1) - Target_traj_y(last_index - 2)) / dt) / dt;
        double acceleration = sqrt(targe_aend_x * targe_aend_x + targe_aend_y * targe_aend_y);

        // if vel or acc on lat point is bigger than the max vel / max acc, rescaling

        if (velocity > max_vel)
        {
            target_vend_x = target_vend_x / velocity * max_vel * 0.85;
            target_vend_y = target_vend_y / velocity * max_vel * 0.85;
        }

        if (acceleration > max_acc)
        {
            targe_aend_x = targe_aend_x / acceleration * max_acc * 0.85;
            targe_aend_y = targe_aend_y / acceleration * max_acc * 0.85;
        }

        fS.setZero();
        fS(0, 0) = Target_traj_x(last_index);
        fS(1, 0) = Target_traj_y(last_index);
        fS(0, 1) = target_vend_x;
        fS(1, 1) = target_vend_y;
        fS(0, 2) = targe_aend_x;
        fS(1, 2) = targe_aend_y;

        ////////////////////////
        // Verification that the target is not both too slow and too close from the drone
        // those 2 conditions are fulfilled, no trajectory is created
        ////////////////////////

        double dist_uav_target = sqrt((UAV_px - Target_traj_x(0)) * (UAV_px - Target_traj_x(0)) + (UAV_py - Target_traj_y(0)) * (UAV_py - Target_traj_y(0)));

        if (dist_uav_target == 0 && velocity < 0.01)
        {
            ROS_INFO("No trjactory created because uav and taget already close and  target is really slow");
            return;
        }

        //////////////////////
        Eigen::VectorXd Time_guess;
        Time_guess = Visible_region.Time_vector;
        for (int i = 0; i < Number_of_points; i++)
        {
            double dist = 0;
            if (i == 0)
            {
                dist = sqrt(pow(Visible_region.sample(0, 1) - UAV_px, 2) + pow(Visible_region.sample(1, 1) - UAV_py, 2));
                Time_guess(i) = std::max(dist / (5 * 0.9), Visible_region.Time_vector(i));
            }
            else
            {
                dist = sqrt(pow(Visible_region.sample(0, i + 1) - Visible_region.sample(0, i), 2) + pow(Visible_region.sample(1, i + 1) - Visible_region.sample(1, i), 2));
                Time_guess(i) = std::max(dist / (5 * 0.9), Visible_region.Time_vector(i));
            }
        }

        // Definition of x_init, the initial guess of the solution for the moving points in the optimization process.
        Eigen::VectorXd X_init(3 * Number_of_points - 2);
        X_init.segment(0, Number_of_points - 1) = Visible_region.sample.row(0).segment(1, Number_of_points - 1);
        X_init.segment(Number_of_points - 1, Number_of_points - 1) = Visible_region.sample.row(1).segment(1, Number_of_points - 1);
        X_init.tail(Number_of_points) = Time_guess;

        // initial value of the weights
        Eigen::VectorXd K_(Number_of_points);
        K_.fill(1);
        Eigen::VectorXd rho_v_(Number_of_points);
        rho_v_.fill(128);
        Eigen::VectorXd rho_a_(Number_of_points);
        rho_a_.fill(128);

        Eigen::VectorXd raw_result;
        Cost_fun_Minimizer Minimizer = Cost_fun_Minimizer(iS, fS, Visible_region.M, Visible_region.A, Visible_region.b, Visible_region.Time_vector, K_, rho_v_, rho_a_);
        ROS_INFO("Minimization begin");
        raw_result = Minimizer.run(X_init);
        ROS_INFO("Minimization End");

        // reShape the solution to be usable
        std::tuple<Eigen::Matrix2Xd, Eigen::VectorXd> result = CF::split_vector(raw_result);
        Eigen::MatrixXd variable_points = std::get<0>(result);
        Eigen::VectorXd ts = std::get<1>(result);
        Eigen::MatrixXd route(3, Visible_region.M + 1);

        // Verification that the trajectory is possible (meaning there are no negative time )
        //(that can happen when trajectory is really small or when the optimization process did not succedd)
        bool all_pos = all_positive(ts);
        std::cout << "Ts" << std::endl;
        std::cout << ts << std::endl;

        if (!all_pos)
        {
            std::cout << "ts" << std::endl;
            std::cout << ts << std::endl;
            // do not create any trajectory this time because initial optimization process did not succeed
            ROS_INFO("Optimization error, no UAV trjactory this time");
            return;
        }
        else
        {

            route.block(0, 1, 2, Visible_region.M - 1) = variable_points;
            route.col(0) = iS.block(0, 0, 3, 1);
            route.col(Visible_region.M) = fS.block(0, 0, 3, 1);
            route.row(2).setZero();

            min_jerk::Trajectory minJerkTraj;
            Minimizer.Cost_fun.jerkOpt.getTraj(minJerkTraj);
            double time = minJerkTraj.getTotalDuration();

            CV::Constraint_Verification Verif = CV::Constraint_Verification();
            Eigen::VectorXd init_psn(2);
            init_psn(0) = UAV_px;
            init_psn(1) = UAV_py;
            Verif.Verify_constraints(max_vel, max_acc, Nbr_verification, ts, Visible_region.A, Visible_region.b, minJerkTraj, init_psn, Visible_region.Index_vec, K_, rho_v_, rho_a_);

            int iterator = 0;
            while (Verif.constraint_ok == false && iterator < 2)
            {
                std::cout << "Constraint violation" << std::endl;
                Eigen::VectorXi New_index_vec = Verif.New_Index_vec;
                Visible_region.Resample(Verif.New_Index_vec, dt, Target_traj_x, Target_traj_y, FOV_dist);

                Time_guess.resize(Visible_region.M);
                Time_guess = Visible_region.Time_vector;
                for (int i = 0; i < Number_of_points; i++)
                {
                    double dist = 0;
                    if (i == 0)
                    {
                        dist = sqrt(pow(Visible_region.sample(0, 1) - UAV_px, 2) + pow(Visible_region.sample(1, 1) - UAV_py, 2));
                        Time_guess(i) = std::max(dist / (5 * 0.85), Visible_region.Time_vector(i));
                    }
                    else
                    {
                        dist = sqrt(pow(Visible_region.sample(0, i + 1) - Visible_region.sample(0, i), 2) + pow(Visible_region.sample(1, i + 1) - Visible_region.sample(1, i), 2));
                        Time_guess(i) = std::max(dist / (5 * 0.85), Visible_region.Time_vector(i));
                    }
                }

                X_init.resize(3 * Visible_region.M - 2);
                X_init.segment(0, Visible_region.M - 1) = Visible_region.sample.row(0).segment(1, Visible_region.M - 1);
                X_init.segment(Visible_region.M - 1, Visible_region.M - 1) = Visible_region.sample.row(1).segment(1, Visible_region.M - 1);
                X_init.tail(Visible_region.M) = Visible_region.Time_vector; // Time_guess * (iterator +1)* 2 ;

                raw_result.resize(3 * Visible_region.M - 2);
                Cost_fun_Minimizer Minimizer2 = Cost_fun_Minimizer(iS, fS, Visible_region.M, Visible_region.A, Visible_region.b, Visible_region.Time_vector, Verif.K_plus, Verif.rho_a_plus, Verif.rho_v_plus);
                raw_result = Minimizer2.run(X_init);

                result = CF::split_vector(raw_result);
                variable_points.resize(2, Visible_region.M - 1);
                variable_points = std::get<0>(result);
                ts.resize(Visible_region.M);
                ts = std::get<1>(result);
                std::cout << "Ts" << ts << std::endl;

                bool all_pos = all_positive(ts);
                std::cout << "Ts" << std::endl;
                std::cout << ts << std::endl;

                if (!all_pos)
                {
                    std::cout << "ts" << std::endl;
                    std::cout << ts << std::endl;
                    // do not create any trajectory this time because initial optimization process did not succeed
                    ROS_INFO("Optimization error, no UAV trjactory this time");
                    return;
                }

                route.resize(3, Visible_region.M + 1);

                route.block(0, 1, 2, Visible_region.M - 1) = variable_points;
                route.col(0) = iS.block(0, 0, 3, 1);
                route.col(Visible_region.M) = fS.block(0, 0, 3, 1);
                route.row(2).setZero();

                Minimizer2.Cost_fun.jerkOpt.getTraj(minJerkTraj);
                time = minJerkTraj.getTotalDuration();

                Verif.Verify_constraints(5, 2.5, 10, ts, Visible_region.A, Visible_region.b, minJerkTraj, init_psn, Visible_region.Index_vec, Verif.K_plus, Verif.rho_a_plus, Verif.rho_v_plus);
                ROS_INFO("constraint verification iteration : %i", iterator);
                ROS_INFO("Constraint verification : %s", Verif.constraint_ok ? "Ok" : "Not Ok");
                iterator = iterator + 1;
                // if(iterator == 2 && Verif.constraint_ok== false){
                //     ROS_INFO("Constraint violation no trajectory created");
                //     return;
                // }
            }

            nav_msgs::Path UAV_path = UAV_Path(minJerkTraj, time);
            UAV_traj_rviz.publish(UAV_path);

            int Point_number = static_cast<int>(time / dt);
            double Traj_time_corrected = dt * Point_number;

            geometry_msgs::Quaternion quat_orientation = Uav_odom->pose.orientation;
            double heading_init = mrs_lib::AttitudeConverter(quat_orientation).getHeading();
            if(use_heading == false){
               heading_init = 0; 
            }

            mrs_msgs::TrajectoryReference UAV_traj = UAV_TrajectoryReference_generatation(Time_init, Traj_time_corrected, minJerkTraj, altitude, dt, Point_number, heading_init);
            if (use_heading == true)
            {
                // std::cout << "Heading init" << heading_init << std::endl;

                // std::cout << "Veloctity" << velocity << std::endl;
                if (velocity > 0.25)
                {
                    Eigen::Vector3d Heading_state;
                    geometry_msgs::Vector3 uav_angular_vel = Uav_odom->velocity.angular;
                    geometry_msgs::Vector3 uav_angular_acc = Uav_odom->acceleration.angular;

                    double heading_init = mrs_lib::AttitudeConverter(quat_orientation).getHeading();
                    double heading_rate_init = mrs_lib::AttitudeConverter(quat_orientation).getHeadingRate(uav_angular_vel);
                    double heading_acc_init = mrs_lib::AttitudeConverter(quat_orientation).getHeadingRate(uav_angular_acc);

                    Heading_state << heading_init, heading_rate_init, heading_acc_init;

                    Heading_calc.Optimize(Heading_state, Target_traj_x, Target_traj_y, UAV_traj);
                }
            }

// Creation of flight corridor plotting in RVIZ
            visualization_msgs::MarkerArray marker_array;
            removeAllMarkers(UAV_corridor_rviz);
            int col_nbr = Visible_region.Vertices.cols();
            for(int i =0; i < col_nbr; i = i+2){
                Eigen::VectorXd Vx=Visible_region.Vertices.col(i);
                Eigen::VectorXd Vy=Visible_region.Vertices.col(i+1);
                visualization_msgs::Marker marker = createPolygonMarker(Vx,Vy,i);
                marker_array.markers.push_back(marker);
                
            }
            UAV_corridor_rviz.publish(marker_array);
            std::cout<< "Vertices" <<std::endl;

            std::cout<< Visible_region.Vertices <<std::endl;

 
            if (setTrajectorySrv(UAV_traj))
            {

                ROS_INFO("[trajectory set");
            }

            ros::Time time_end = ros::Time::now();
            double calculation_time = (time_end - time_init).toSec();
            ROS_INFO("Le temps de calcul dans ROS est %f", calculation_time);
        }
    }

    bool callbackActivate([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        res.success = true;
        res.message = "activated";
        return true;
    }

    bool setTrajectorySrv(const mrs_msgs::TrajectoryReference &trajectory)
    {
        mrs_msgs::TrajectoryReferenceSrv srv;
        srv.request.trajectory = trajectory;

        bool success = service_client_trajectory_.call(srv);

        if (success)
        {

            if (!srv.response.success)
            {
                ROS_ERROR_STREAM_THROTTLE(1.0, "[TrajectoryRandomFlier]: service call for setting trajectory failed: " << srv.response.message);
                return false;
            }
            else
            {
                return true;
            }
        }
        else
        {
            ROS_ERROR_THROTTLE(1.0, "[TrajectoryRandomFlier]: service call for setting trajectory failed");
            return false;
        }
    }
};

int main(int argc, char **argv)
{
    std::cout << "Node creation" << std::endl;
    ros::init(argc, argv, "UAV_trajectory_optimizer_node");
    
    float requested_alt = 0.0;
    for (int i = 0; i < argc; ++i) {
        std::cout << "checkstring" << std::endl;
        if (std::string(argv[i]) == "--altitude") {
            // Retrieve the value of the argument
            requested_alt = std::stof(argv[i + 1]);
        }
    }
    
    UAV_trajectory_optimizer_node Node(requested_alt);
    ros::spin();
}

