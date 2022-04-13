//
// Created by usl on 3/28/22.
//


#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv){
    ros::init(argc, argv, "ros_camera_imu_lidar_calibration_bag_generator");
    ros::NodeHandle nh("~");

    /// Our topics (IMU and LIDAR)
    std::string topic_imu;
    std::string topic_lidar;
    std::string topic_camera;
    nh.param<std::string>("topic_imu", topic_imu, "/imu");
    nh.param<std::string>("topic_lidar", topic_lidar, "/lidar");
    nh.param<std::string>("topic_camera", topic_camera, "/camera");

    /// Location of the ROS bag we want to read in
    std::string path_to_bag;
    std::string path_out_bag;
    nh.param<std::string>("path_bag", path_to_bag, "/home/usl/datasets/ouster_vectornav.bag");
    ROS_INFO_STREAM("ROS BAG PATH is: " << path_to_bag.c_str());
    nh.param<std::string>("path_out_bag", path_out_bag, "/home/usl/datasets/ouster_vectornav.bag");
    ROS_INFO_STREAM("ROS Output BAG PATH is: " << path_out_bag.c_str());

    /// Get our start location and how much of the bag we want to play
    /// Make the bag duration < 0 to just process to the end of the bag
    double bag_start, bag_durr;
    nh.param<double>("bag_start", bag_start, 0);
    nh.param<double>("bag_durr", bag_durr, -1);
    ROS_INFO_STREAM("bag start: " << bag_start);
    ROS_INFO_STREAM("bag duration: " << bag_durr);

    /// Load rosbag here, and find messages we can play
    rosbag::Bag bag;
    bag.open(path_to_bag, rosbag::bagmode::Read);

    /// We should load the bag as a view
    /// Here we go from beginning of the bag to the end of the bag
    rosbag::View view_full;
    rosbag::View view;

    /// Start a few seconds in from the full view time
    /// If we have a negative duration then use the full bag length
    view_full.addQuery(bag);
    ros::Time time_init = view_full.getBeginTime();
    time_init += ros::Duration(bag_start);
    ros::Time time_finish =
            (bag_durr < 0) ? view_full.getEndTime() : time_init + ros::Duration(bag_durr);
    ROS_INFO_STREAM("Time start = " << time_init.toSec());
    ROS_INFO_STREAM("Time end = " << time_finish.toSec());
    view.addQuery(bag, time_init, time_finish);

    /// Check to make sure we have data to play
    if (view.size() == 0) {
        ROS_ERROR_STREAM("No messages to play on specified topics.  Exiting.");
        ros::shutdown();
        return EXIT_FAILURE;
    }
    bool first_run = true;
    double timestamp_first_frame = 0;
    double timestamp_current = 0;
    double timestamp_first_m = 0;
    double timestamp_current_m = 0;

    ///
    rosbag::Bag* bag_out;
    bag_out = new rosbag::Bag(path_out_bag.c_str(), rosbag::bagmode::Write);

    for (const rosbag::MessageInstance& m : view) {
        /// If ROS wants us to stop, break out
        if (!ros::ok())
            break;
        sensor_msgs::Imu::ConstPtr s_imu = m.instantiate<sensor_msgs::Imu>();
        sensor_msgs::PointCloud2::ConstPtr s_lidar = m.instantiate<sensor_msgs::PointCloud2>();
        sensor_msgs::Image::ConstPtr s_image = m.instantiate<sensor_msgs::Image>();
        std::string sensor_name = "";
        if (s_imu != nullptr && m.getTopic() == topic_imu || s_lidar != nullptr && m.getTopic() == topic_lidar
            || s_image != nullptr && m.getTopic() == topic_camera) {
            if(s_imu != nullptr) {
                bag_out->write("/vectornav/IMU", (*s_imu).header.stamp, s_imu);
            } else if (s_image != nullptr) {
                bag_out->write("/pylon_camera_node/image_raw", (*s_image).header.stamp, s_image);
            } else {
                bag_out->write("/os_cloud_node/points", (*s_lidar).header.stamp, s_lidar);
            }
        }
    }
    bag_out->close();
    ROS_INFO_STREAM("Created bag file");
    return EXIT_SUCCESS;
}