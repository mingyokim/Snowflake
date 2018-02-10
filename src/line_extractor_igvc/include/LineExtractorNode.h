/*
 * Created By: Min Gyo Kim
 * Created On: January 20, 2018
 * Description: Header file for Line Extractor Node
 */

#ifndef PROJECT_LINEEXTRACTOR_H
#define PROJECT_LINEEXTRACTOR_H

#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sb_utils.h>
#include <math.h>
#include <string>
#include "DBSCAN.h"
#include "Regression.h"
#include <mapping_igvc/LineObstacle.h>

class LineExtractorNode {
public:
    std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;

    LineExtractorNode(int argc, char **argv, std::string node_name);

    // main entry function
    void extractLines();
private:
    ros::Subscriber subscriber;
    ros::Publisher publisher;

    DBSCAN dbscan;
    Regression regression;



    pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr;

    void pclCallBack(const sensor_msgs::PointCloud2ConstPtr input);
    std::vector<mapping_igvc::LineObstacle> vectorsToMsgs(std::vector<Eigen::VectorXf> vectors);
    mapping_igvc::LineObstacle vectorToLineObstacle(Eigen::VectorXf vector, unsigned int clusterIndex);
    void getClusterXRange(double &xmin, double &xmax, unsigned int clusterIndex);
};


#endif //PROJECT_LINEEXTRACTOR_H
