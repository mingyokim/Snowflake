/*
 * Created By: Min Gyo Kim
 * Created On: January 20, 2018
 * Description: Class Implementation of DBSCAN (Density-based clustering)
 */

#include <DBSCAN.h>

DBSCAN::DBSCAN(int min_neighbours, int radius) {
    this->_min_neighbors = min_neighbours;
    this->_radius = radius;
    this->_clusters = vector<vector<pcl::PointXYZ>>();
}

void DBSCAN::setMinNeighbours(int new_min_neighour) {
    this->_min_neighbors = new_min_neighour;
}

void DBSCAN::setRadius(float new_radius) {
    this->_radius = new_radius;
}

vector<vector<pcl::PointXYZ>> DBSCAN::findClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr) {
    this->_pcl = *pclPtr;

    for (unsigned int i = 0; i < this->_pcl.size(); i++) {
        if( isPointVisited(i) ) {
            continue;
        }
        if( isCore(i) ) {
            vector<pcl::PointXYZ> cluster = vector<pcl::PointXYZ>();
            pcl::PointXYZ currentPoint = this->_pcl[i];
            cluster.push_back(currentPoint);
            this->_visited.insert({i,true});
            expand(i, cluster);
            this->_clusters.push_back(cluster);
        }
    }

    return this->_clusters;
}

bool DBSCAN::isCore(int centerIndex) {
    int numNeighbours = 0;
    for (unsigned int i = 0; i < this->_pcl.size(); i++ ) {
        pcl::PointXYZ currentPoint = this->_pcl[i];
        if ( centerIndex != i && dist(currentPoint, this->_pcl[centerIndex]) <= this->_radius) {
            numNeighbours++;
        }
    }

    if (numNeighbours >= this->_min_neighbors ) return true;
    return false;
}

void DBSCAN::expand(int centerIndex, vector<pcl::PointXYZ> &cluster) {
//    vector<unsigned int> newCluster;
    this->_expanded.insert({centerIndex,true});
    for (unsigned int i = 0; i < this->_pcl.size(); i++) {
        if( i == centerIndex )  continue;
        pcl::PointXYZ currentPoint = this->_pcl[i];
        if (dist(currentPoint, this->_pcl[centerIndex]) <= this->_radius)  {
            if( !isPointVisited(i) ) {
//                newCluster.push_back(i);
                cluster.push_back(currentPoint);
                this->_visited.insert({i,true});
            }
            if (isCore(i) && !isPointExpanded(i)) {
                expand(i,cluster);
                this->_expanded.insert({i,true});
            }
        }
    }

//    for (unsigned int i = 0; i < newCluster.size(); i++) {
//        pcl::PointXYZ currentPoint = this->_pcl[newCluster[i]];
//        cluster.push_back(currentPoint);
//        if (isCore(i)) {
//            expand(i,cluster);
//        }
//    }

    return;
}

float DBSCAN::dist(pcl::PointXYZ p1, pcl::PointXYZ p2) {
    float dx = abs( p1.x - p2.x);
    float dy = abs( p1.y - p2.y);
    return sqrt(pow(dx,2)+pow(dy,2));
}

bool DBSCAN::isPointVisited(int pIndex) {
    return this->_visited.find(pIndex) != this->_visited.end();
}

bool DBSCAN::isPointExpanded(int pIndex) {
    return this->_expanded.find(pIndex) != this->_expanded.end();
}
