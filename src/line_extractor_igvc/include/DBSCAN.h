/*
 * Created By: Min Gyo Kim
 * Created On: January 20, 2018
 * Description: Class declaration for DBSCAN (Density-based clustering)
 */

#ifndef PROJECT_DBSCAN_H
#define PROJECT_DBSCAN_H

#include <geometry_msgs/Point.h>

using Point = geometry_msgs::Point;
using namespace std;

class DBSCAN {
    vector<Point> _points;
    vector<vector<Point>> _clusters;

    // TODO: fine-tune parameters with real data
    int _min_neighbors = 5;
    float _radius = 5;

    void expand(Point center, vector<Point> &cluster);
    bool isCore(Point point);

public:
    DBSCAN(int min_neighbours, int radius);
    vector<vector<Point>> findClusters(vector<Point> points);
    void setMinNeighbours(int new_min_neighour);
    void setRadius(float new_radius);
    float dist(Point p1, Point p2);
};


#endif //PROJECT_DBSCAN_H
