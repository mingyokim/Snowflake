/*
 * Created By: Min Gyo Kim
 * Created On: January 27th 2018
 * Description: GTest for DBSCAN implementation
 */

#include <gtest/gtest.h>
#include <DBSCAN.h>

int min_neighbours = 4;
int radius = 5;
DBSCAN dbscan(min_neighbours, radius);

TEST(DBSCAN, TestClusterTwoHorizontalLines){
    pcl::PointCloud<pcl::PointXYZ> pcl;

//    first line
    int y1 = 10;
    for( int x = -10; x <= 10; x++ ) {
        pcl::PointXYZ p;
        p.x = x;
        p.y = y1;
        pcl.push_back(p);
    }

    int y2 = -10;
    for( int x = -10; x <= 10; x++ ) {
        pcl::PointXYZ p;
        p.x = x;
        p.y = y2;
        pcl.push_back(p);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr = pcl.makeShared();

    vector<vector<pcl::PointXYZ>> clusters = dbscan.findClusters(pclPtr);
    EXPECT_EQ(2, clusters.size());
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}