/*
 * Created By: Min Gyo Kim
 * Created On: January 20, 2018
 * Description: Header file for Line Extractor Node
 */

#ifndef PROJECT_LINEEXTRACTOR_H
#define PROJECT_LINEEXTRACTOR_H

#include <iostream>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud.h>
#include <ros/ros.h>
#include <sb_utils.h>
#include <math.h>
#include <string>

class LineExtractorNode {
public:
    LineExtractorNode(int argc, char **argv, std::string node_name);

    // main entry function
    void extractLines();
private:
    ros::Subscriber subscriber;
    ros::Publisher publisher;

    sensor_msgs::PointCloud pointCloud;

    void pclCallBack(const sensor_msgs::PointCloud pointCloud);
};


#endif //PROJECT_LINEEXTRACTOR_H
