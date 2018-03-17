//
// Created by min on 10/03/18.
//

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "../test/TestUtils.h"

std::vector<float> first_line;
std::vector<float> second_line;

sensor_msgs::PointCloud2 generatePclMessage();

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_pcl_generator_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Rate loop_rate = 1;
    ros::Publisher publisher = nh.advertise<sensor_msgs::PointCloud2>("input_pointcloud", 1);

    std::string first_line_param = "first_line";
    std::vector<float> default_first_line = {50, 0, -0.01};
    private_nh.param(first_line_param, first_line, default_first_line);

    std::string second_line_param = "second_line";
    std::vector<float> default_second_line = {0, 0, -0.01};
    private_nh.param(second_line_param, second_line, default_second_line);

    sensor_msgs::PointCloud2 msg_to_publish = generatePclMessage();
    msg_to_publish.header.frame_id = "line_extractor_test";

    while(ros::ok()) {
        publisher.publish(msg_to_publish);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

sensor_msgs::PointCloud2 generatePclMessage() {
    float x_min   = -50;
    float x_max   = 50;
    float x_delta = 0.1;

    // coefficients is the same as the one in LineObstacle message
    LineExtractor::TestUtils::LineArgs args(
            first_line, x_min, x_max, x_delta);

    float max_noise_x = 5;
    float max_noise_y = 5;

    // Generate a single PointCloud with noise
    pcl::PointCloud<pcl::PointXYZ> pcl;
    LineExtractor::TestUtils::addLineToPointCloud(
            args, pcl, max_noise_x, max_noise_y);

    args.coefficients = second_line;
    LineExtractor::TestUtils::addLineToPointCloud(
            args, pcl, max_noise_x, max_noise_y);

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(pcl, msg);

    return msg;
}