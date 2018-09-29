/*
 * Created By: Min Gyo Kim
 * Created On: September 29, 2018
 * Description: Header file for Ball Extractor Node
 */

#ifndef BALL_EXTRACTOR_IGVC_NODE_H
#define BALL_EXTRACTOR_IGVC_NODE_H

#include <DBSCAN.h>
#include <RvizUtils.h>
#include <iostream>
#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sb_utils.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

class BallExtractorNode {
  public:
    /*
     * @clusters stores the output from DBSCAN
     */
    std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;

    BallExtractorNode(int argc, char** argv, std::string node_name);

    // main entry function
    void extractBall();

    /*
     * @clusters: input clusters
     * @cluster_points: vector to store converted cluster points
     * @colors: vector to store color for each cluster point
     * This function takes in @clusters, assigns a different color
     * for each cluster, and converts each point to geometry_msgs::Point.
     * It pushes the points and colors to the back of @cluster_points
     * and @colors.
     */
    static void convertClustersToPointsWithColors(
    std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters,
    std::vector<geometry_msgs::Point>& cluster_points,
    std::vector<std_msgs::ColorRGBA>& colors);

  private:
    ros::Subscriber subscriber;
    ros::Publisher publisher;
    ros::Publisher rviz_cluster_publisher;

    /*
     * @minNeighbours is a hyperparameter to DBSCAN that determines
     * how many neighbours are required for a point to be considered
     * a 'core' point
     */
    int minNeighbours;

    /*
     * @radius is a hyperparameter to DBSCAN that determines
     * how close the points have to be together in order for them to
     * be considered as 'core' points
     */
    float radius;

    /*
     * scale of the marker points
     */
    float scale;

    /*
     * frame_id for rviz markers
     */
    std::string frame_id;

    /*
     * @pclPtr stores the pointer to the PCL PointCloud after it has
     * been converted from sensor_msgs PointCloud2
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr;

    /*
     * The callback function is called whenever the node receives a
     * PointCloud message. It converts sensor_msgs PointCloud2 pointer
     * to PCL PointCloud pointer and then extracts lines from the PointCloud
     */
    void pclCallBack(const sensor_msgs::PointCloud2ConstPtr processed_pcl);

    /*
     * This function makes a Marker for all points in @clusters
     * with a different color for each cluster and publishes a message
     * to rviz for visualization at "~/debug/clusters".
     */
    void visualizeClusters();

    /*
     * Get the minimum and maximum value of x value of all points in a cluster
     * @cluster_index: the index of cluster of interest in @clusters
     */
    void
    getClusterXRange(double& xmin, double& xmax, unsigned int cluster_index);

    /*
     * Checks whether or not all the params we are getting from NodeHandler are
     * valid
     * params being checked: degree_polynomial, lambda, min_neighbours, radius
     */
    bool areParamsInvalid();
};

#endif // PROJECT_BALL_EXTRACTOR_H