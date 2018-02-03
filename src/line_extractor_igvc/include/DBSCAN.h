/*
 * Created By: Min Gyo Kim
 * Created On: January 20, 2018
 * Description: Class declaration for DBSCAN (Density-based clustering)
 */

#ifndef PROJECT_DBSCAN_H
#define PROJECT_DBSCAN_H

#include <tr1/unordered_map>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

using namespace std;
using namespace std::tr1;

class DBSCAN {
    pcl::PointCloud<pcl::PointXYZ> _pcl;
    vector<pcl::PointCloud<pcl::PointXYZ>> _clusters;
    unordered_map<unsigned int,bool> _clustered;
    unordered_map<unsigned int,bool> _expanded;
    unordered_map<unsigned int,bool> _is_core;

    // TODO: fine-tune parameters with real data
    int _min_neighbors = 5;
    float _radius = 5;

    void expand(unsigned int centerPointIndex, pcl::PointCloud<pcl::PointXYZ> &cluster);
    bool isCore(unsigned int centerPointIndex);

public:
    DBSCAN(int min_neighbours=5, int radius=5);
    vector<pcl::PointCloud<pcl::PointXYZ>> findClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr);
    void setMinNeighbours(int new_min_neighour);
    void setRadius(float new_radius);

private:
    float dist(pcl::PointXYZ p1, pcl::PointXYZ p2);
    bool isPointVisited(unsigned int pIndex);
    bool isPointExpanded(unsigned int pIndex);
//    bool arePointsEqual(Point p1, Point p2);
};


#endif //PROJECT_DBSCAN_H
