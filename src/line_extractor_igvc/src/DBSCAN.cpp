/*
 * Created By: Min Gyo Kim
 * Created On: January 20, 2018
 * Description: Class Implementation of DBSCAN (Density-based clustering)
 */

#include <DBSCAN.h>

DBSCAN::DBSCAN(int min_neighbours, int radius) {
    this->_min_neighbors = min_neighbours;
    this->_radius = radius;
    this->_clusters = vector<pcl::PointCloud<pcl::PointXYZ>>();
}

void DBSCAN::setMinNeighbours(int new_min_neighour) {
    this->_min_neighbors = new_min_neighour;
}

void DBSCAN::setRadius(float new_radius) {
    this->_radius = new_radius;
}

vector<pcl::PointCloud<pcl::PointXYZ>> DBSCAN::findClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr) {
    this->_pcl = *pclPtr;

    findNeighbors();

    for (unsigned int i = 0; i < this->_pcl.size(); i++) {
        if( isPointVisited(i) ) {
            continue;
        }
        if( isCore(i) ) {
            pcl::PointCloud<pcl::PointXYZ> cluster = pcl::PointCloud<pcl::PointXYZ>();
            pcl::PointXYZ currentPoint = this->_pcl[i];
            cluster.push_back(currentPoint);
            this->_clustered.insert({i,true});
            expand(i, cluster);
            this->_clusters.push_back(cluster);
        }
    }

    return this->_clusters;
}

bool DBSCAN::isCore(unsigned int centerIndex) {
    unordered_map<unsigned int,vector<unsigned int>>::const_iterator it = this->_neighbors.find(centerIndex);
    return (it->second).size() >= this->_min_neighbors;
}

void DBSCAN::expand(unsigned int centerIndex, pcl::PointCloud<pcl::PointXYZ> &cluster) {
    this->_expanded.insert({centerIndex,true});

    vector<unsigned int> neighbors = this->_neighbors.find(centerIndex)->second;
    for (unsigned int i = 0; i < neighbors.size(); i++) {
        pcl::PointXYZ currentPoint = this->_pcl[i];
        unsigned int currentIndex = neighbors[i];
        if( !isPointVisited(currentIndex) ) {
            cluster.push_back(currentPoint);
            this->_clustered.insert({currentIndex,true});
        }
        if ( !isPointExpanded(currentIndex) && isCore(currentIndex) ) {
            expand(currentIndex,cluster);
            this->_expanded.insert({currentIndex,true});
        }
    }

    return;
}

void DBSCAN::findNeighbors() {
    for( unsigned int i = 0; i < this->_pcl.size(); i++ ) {
        vector<unsigned int> neighbors;
        pcl::PointXYZ currentPoint = this->_pcl[i];
        for( unsigned int j = 0; j < this->_pcl.size(); j++ ) {
            if( i==j )  continue;
            pcl::PointXYZ neighborPoint = this->_pcl[j];
            if( dist(currentPoint, neighborPoint) <= this->_radius ) {
                neighbors.push_back(j);
            }
        }

        this->_neighbors.insert({i,neighbors});
    }
}

float DBSCAN::dist(pcl::PointXYZ p1, pcl::PointXYZ p2) {
    float dx = abs( p1.x - p2.x);
    float dy = abs( p1.y - p2.y);
    return sqrt(pow(dx,2)+pow(dy,2));
}

bool DBSCAN::isPointVisited(unsigned int pIndex) {
    return this->_clustered.find(pIndex) != this->_clustered.end();
}

bool DBSCAN::isPointExpanded(unsigned int pIndex) {
    return this->_expanded.find(pIndex) != this->_expanded.end();
}
