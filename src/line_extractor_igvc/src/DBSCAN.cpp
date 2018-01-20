/*
 * Created By: Min Gyo Kim
 * Created On: January 20, 2018
 * Description: Class Implementation of DBSCAN (Density-based clustering)
 */

#include <DBSCAN.h>

DBSCAN::DBSCAN(int min_neighbours=5, int radius=5) {
    this->_min_neighbors = min_neighbours;
    this->_radius = radius;
    this->_clusters = vector<vector<Point>>();
}

void DBSCAN::setMinNeighbours(int new_min_neighour) {
    this->_min_neighbors = new_min_neighour;
}

void DBSCAN::setRadius(float new_radius) {
    this->_radius = new_radius;
}

vector<vector<Point>> DBSCAN::findClusters(vector<Point> points) {
    this->_points = points;

//    unordered_map
//
//    for (vector<Point>::iterator it = this->_points.begin(); it != this->_points.end(); ++it) {
//        Point currentPoint = *it;
//        if (currentPoint != point && dist(currentPoint, point) < this->_radius) {
//            numNeighbours++;
//        }
//    }

    return this->_clusters;
}

bool DBSCAN::isCore(Point point) {
    int numNeighbours = 0;
    for (vector<Point>::iterator it = this->_points.begin(); it != this->_points.end(); ++it) {
        Point currentPoint = *it;
        if (currentPoint != point && dist(currentPoint, point) < this->_radius) {
            numNeighbours++;
        }
    }

    if (numNeighbours >= this->_min_neighbors ) return true;
    return false;
}

void DBSCAN::expand(Point center, vector<Point> &cluster) {
    vector<Point> newCluster;
    for (vector<Point>::iterator it = this->_points.begin(); it != this->_points.end(); ++it) {
        Point currentPoint = *it;
        if (currentPoint != center) continue;
        if (dist(currentPoint, center) < this->_radius)  {
            newCluster.push_back(currentPoint);
        }
    }

    for (vector<Point>::iterator it = newCluster.begin(); it != newCluster.end(); ++it) {
        Point currentPoint = *it;
        cluster.push_back(currentPoint);
        if (isCore(currentPoint)) {
            expand(currentPoint,cluster);
        }
    }

    return;
}

float DBSCAN::dist(Point p1, Point p2) {
    float dx = abs( p1.x - p2.x);
    float dy = abs( p1.y - p2.y);
    return sqrt(pow(dx,2)+pow(dy,2));
}

