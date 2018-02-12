/*
 * Created By: Min Gyo Kim
 * Created On: January 20, 2018
 * Description: Header file for Line Extractor Node
 */

#ifndef PROJECT_LINEEXTRACTOR_H
#define PROJECT_LINEEXTRACTOR_H

#include "DBSCAN.h"
#include "Regression.h"
#include <iostream>
#include <mapping_igvc/LineObstacle.h>
#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sb_utils.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

class LineExtractorNode {
  public:
    std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;

    LineExtractorNode(int argc, char** argv, std::string node_name);

    // main entry function
    void extractLines();

  private:
    ros::Subscriber subscriber;
    ros::Publisher publisher;

    DBSCAN dbscan;
    Regression regression;

    int degreePoly;
    float lambda;
    int minNeighbours;
    float radius;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr;

    void pclCallBack(const sensor_msgs::PointCloud2ConstPtr input);

    /*
     * Convert a list of vectors to a list of LineObstacle message
     */
    std::vector<mapping_igvc::LineObstacle>
    vectorsToMsgs(std::vector<Eigen::VectorXf> vectors);

    /*
     * Convert a vector to LineObstacle message
     */
    mapping_igvc::LineObstacle vectorToLineObstacle(Eigen::VectorXf vector,
                                                    unsigned int clusterIndex);

    /*
     * Get the minimum and maximum value of x value of all points in a cluster
     * @clusterIndex: the index of cluster of interest in @clusters
     */
    void
    getClusterXRange(double& xmin, double& xmax, unsigned int clusterIndex);

    /*
     * Checks whether or not all the params we are getting from NodeHandler are
     * valid
     * params being checked: degree_polynomial, lambda, min_neighbours, radius
     */
    bool areParamsInvalid();
};

#endif // PROJECT_LINEEXTRACTOR_H
