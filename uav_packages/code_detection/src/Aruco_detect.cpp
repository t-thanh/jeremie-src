#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include<vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_msgs/Bool.h"

#include <chrono>
using namespace std::chrono;

geometry_msgs::PoseStamped Undetected_target_msgs(const sensor_msgs::ImageConstPtr& img_msg){
    // Return a Pose_stamped msg with no frame_id (empty string) and the x value set to 1000
    // So that a msg can be published on the topic but this will not be taken into account.
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = img_msg->header.stamp; 
        pose_msg.pose.position.z = 10000;  // Set the translation along the x-axis
        
        return pose_msg;


}



geometry_msgs::PoseStamped Target_relative_Posestamped_msg(const sensor_msgs::ImageConstPtr& img_msg, const cv::Vec3d& rvec, const cv::Vec3d& tvec) {
    // Create a PosetStamped message with the position and rotation of the target in the Parent frame
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = img_msg->header.stamp; // Set the timestamp to the acquisition time of image
    pose_msg.header.frame_id = img_msg->header.frame_id;   // Set the frame ID to the camera frame
    pose_msg.pose.position.x = tvec[0];  // Set the translation along the x-axis
    pose_msg.pose.position.y = tvec[1];  // Set the translation along the y-axis
    pose_msg.pose.position.z = tvec[2];  // Set the translation along the z-axis

    // Convert the rotation vector to a quaternion
    cv::Mat rot_mat;
    cv::Rodrigues(rvec, rot_mat);
    //std::cout << "Rotation Matrix: " << rot_mat << "degree" << std::endl;

    tf2::Quaternion quat;
    tf2::Matrix3x3 rot_mat_tf(rot_mat.at<double>(0, 0), rot_mat.at<double>(0, 1), rot_mat.at<double>(0, 2),
                              rot_mat.at<double>(1, 0), rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2),
                              rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2));
    rot_mat_tf.getRotation(quat);

    pose_msg.pose.orientation = tf2::toMsg(quat);
    return pose_msg;
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg, bool view_image,
                   const cv::Ptr<cv::aruco::DetectorParameters>& detectorParams,
                   const cv::Ptr<cv::aruco::Dictionary>& dictionary,
                   const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
                   ros::Publisher topic_pub) {

              
    cv::Mat img;
    try{
        img = cv_bridge::toCvShare(msg, "bgr8")->image;

    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    //if (view_image) {
        cv::Mat outputImage = img.clone();
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    //}

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);

    // std::cout << "Center position:" << std::endl;

    // for (const auto& corners : markerCorners) {
    //     float sum_x = 0, sum_y = 0;
    //     for (const auto& corner : corners) {
    //         sum_x += corner.x;
    //         sum_y += corner.y;
    //     }
    //     float center_x = sum_x / 4;
    //     float center_y = sum_y / 4;
    //     std::cout << "x: " << center_x << ", y: " << center_y << std::endl;
    // }

    //auto end = high_resolution_clock::now();
    //auto elapsed_time = duration_cast<milliseconds>(end - start);

    if (markerIds.size() > 0){
        ROS_INFO("%d Aruco code(s) detected: ", markerIds.size());


        for(int i = 0; i < markerIds.size(); i++) {
            ROS_INFO("size is 0.708 MArker ID is %d", markerIds[i]);
        }
        
            // To be updated because we should not broadcast all markers....
        
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.708, cameraMatrix, distCoeffs, rvecs, tvecs);
            //std::cout << "vector of Euler angles: " <<<< "degree" << std::endl;

        for (int i = 0; i < markerIds.size(); ++i) {

            geometry_msgs::PoseStamped Relative_Target_pose;

            Relative_Target_pose = Target_relative_Posestamped_msg(msg,rvecs[i],  tvecs[i]);
            //std::cout << "vector of Euler angles: " << rvecs[i] * 360 /(2*3.1415) << "degree" << std::endl;
            

            topic_pub.publish(Relative_Target_pose);

            //ROS_INFO("Time of the position msg is  %d", Relative_Target_pose.header.stamp);

            if (view_image) {
                    cv::aruco::drawAxis(outputImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.5);
                }
        }

    }else {
        ROS_INFO("No Aruco Code detected");
        geometry_msgs::PoseStamped Undetected_target;
        Undetected_target = Undetected_target_msgs(msg);
        topic_pub.publish(Undetected_target);

    }

    //std::cout << "Elapsed time: " << elapsed_time.count() << " ms" << std::endl;

    if (view_image) {
            cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
            int width = outputImage.cols;
            int height = outputImage.rows;

            std::cout << "Largeur de l'image : " << width << " pixels" << std::endl;
            std::cout << "Hauteur de l'image : " << height << " pixels" << std::endl;
           
            cv::resize(outputImage, outputImage, cv::Size(853, 480));

            cv::imshow("Image window", outputImage);
            cv::waitKey(1);
        }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_viewer");
    bool view_image = false;

    if (argc > 1 && std::string(argv[1]) == "view_image") {
        view_image = true;
    }

    std::cout << "view_image = " << view_image << std::endl;

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    ros::Publisher topic_pub = nh.advertise<geometry_msgs::PoseStamped>("uav/Target_Relative_Pose", 10);


    // Set the camera parameters 
    ///// D435
    // Camera matrix
        cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        cameraMatrix.at<double>(0, 0) = 924.2758995009278;
        cameraMatrix.at<double>(1, 1) = 924.2759313476419;
        cameraMatrix.at<double>(0, 2) = 640.5;
        cameraMatrix.at<double>(1, 2) = 360.5;

        // Distortion coefficients
        cv::Mat distCoeffs = cv::Mat::zeros(1,5, CV_64F);
    //////
    /////

    // ////Fisheye camera
    std::cout << "dow_rgb" << std::endl;
    // //// camera matrix
    // cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    // cameraMatrix.at<double>(0, 0) = 0.0004988748811511915;
    // cameraMatrix.at<double>(1, 1) = 0.0004988748811511915;
    // cameraMatrix.at<double>(0, 2) = 376.5;
    // cameraMatrix.at<double>(1, 2) = 240.5;
    // cameraMatrix.at<double>(2, 2) = 1.0;
    // //////////
    // // dist coefs 
    // cv::Mat distCoeffs = cv::Mat::zeros(1, 5, CV_32F);
    // /////



    // Set detector parameters
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();

    // Set Aruco dictionary
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    image_transport::Subscriber sub = it.subscribe("/uav1/down_rgbd/color/image_raw", 1, boost::bind(imageCallback, _1, view_image, detectorParams, dictionary, cameraMatrix, distCoeffs, topic_pub));
    
    // POUR la vraie camera 
    //image_transport::Subscriber sub = it.subscribe("/uav1/down_rgbd/color/image_raw", 1, boost::bind(imageCallback, _1, view_image, detectorParams, dictionary, cameraMatrix, distCoeffs, topic_pub));


    if (view_image) {
        cv::namedWindow("Image window");

    }

    ros::spin();

    if (view_image) {
        cv::destroyWindow("Image window");
    }

    return 0;
}
