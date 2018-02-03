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
    unordered_map<unsigned int,bool>::const_iterator it = this->_is_core.find(centerIndex);
    if( it != this->_is_core.end() ) {
        return it->second;
    }

    unsigned int numNeighbours = 0;
    for (unsigned int i = 0; i < this->_pcl.size(); i++ ) {
        pcl::PointXYZ currentPoint = this->_pcl[i];
        if ( centerIndex != i && dist(currentPoint, this->_pcl[centerIndex]) <= this->_radius) {
            numNeighbours++;
            if (numNeighbours >= this->_min_neighbors ) {
                this->_is_core.insert({centerIndex,true});
                return true;
            }
        }
    }

    this->_is_core.insert({centerIndex,false});
    return false;
}

void DBSCAN::expand(unsigned int centerIndex, pcl::PointCloud<pcl::PointXYZ> &cluster) {
    this->_expanded.insert({centerIndex,true});
    for (unsigned int i = 0; i < this->_pcl.size(); i++) {
        if( i == centerIndex )  continue;
        pcl::PointXYZ currentPoint = this->_pcl[i];
        if (dist(currentPoint, this->_pcl[centerIndex]) <= this->_radius)  {
            if( !isPointVisited(i) ) {
                cluster.push_back(currentPoint);
                this->_clustered.insert({i,true});
            }
            if ( !isPointExpanded(i) && isCore(i) ) {
                expand(i,cluster);
                this->_expanded.insert({i,true});
            }
        }
    }

    return;
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
