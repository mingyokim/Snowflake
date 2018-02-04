/*
 * Created By: Min Gyo Kim
 * Created On: January 27th 2018
 * Description: GTest for DBSCAN implementation
 */

#include <gtest/gtest.h>
#include <DBSCAN.h>

TEST(DBSCAN, ClusterTwoNearPoints) {
    int min_neighbours = 1;
    int radius = 5;
    DBSCAN dbscan(min_neighbours, radius);

    pcl::PointCloud<pcl::PointXYZ> pcl;

//    Test two near points
    pcl::PointXYZ p1;
    p1.x = 1;
    p1.y = 1;
    pcl.push_back(p1);

    pcl::PointXYZ p2;
    p2.x = 1.1;
    p2.y = 1.1;
    pcl.push_back(p2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr = pcl.makeShared();

    vector<pcl::PointCloud<pcl::PointXYZ>> clusters = dbscan.findClusters(pclPtr);
    EXPECT_EQ(1, clusters.size());
    EXPECT_EQ(2, clusters[0].size());
}

TEST(DBSCAN, TestClusterTwoFarPoints){
    int min_neighbours = 1;
    int radius = 5;
    DBSCAN dbscan(min_neighbours, radius);

    pcl::PointCloud<pcl::PointXYZ> pcl;

    //    Test two far points
    pcl::PointXYZ p1;
    p1.x = 1;
    p1.y = 1;
    pcl.push_back(p1);

    pcl::PointXYZ p3;
    p3.x = 10;
    p3.y = 10;
    pcl.push_back(p3);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr = pcl.makeShared();

    vector<pcl::PointCloud<pcl::PointXYZ>> clusters = dbscan.findClusters(pclPtr);
    EXPECT_EQ(0, clusters.size());
}

TEST(DBSCAN, TestExpandCluster){
    int min_neighbours = 2;
    int radius = 5;
    DBSCAN dbscan(min_neighbours, radius);

    pcl::PointCloud<pcl::PointXYZ> pcl;

    pcl::PointXYZ p1;
    p1.x = 1;
    p1.y = 1;
    pcl.push_back(p1);

    pcl::PointXYZ p2;
    p2.x = 2;
    p2.y = 1;
    pcl.push_back(p2);

    pcl::PointXYZ p3;
    p3.x = 3;
    p3.y = 1;
    pcl.push_back(p3);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr = pcl.makeShared();

    vector<pcl::PointCloud<pcl::PointXYZ>> clusters = dbscan.findClusters(pclPtr);
    EXPECT_EQ(1, clusters.size());
    EXPECT_EQ(3, clusters[0].size());
}

TEST(DBSCAN, TestClusterTwoShortHorizontalLines){
    int min_neighbours = 2;
    int radius = 5;
    DBSCAN dbscan(min_neighbours, radius);

    pcl::PointCloud<pcl::PointXYZ> pcl;

//    first line
    int y1 = 10;
    for( int x = 0; x <= 3; x++ ) {
        pcl::PointXYZ p;
        p.x = x;
        p.y = y1;
        pcl.push_back(p);
    }
//second line
    int y2 = -10;
    for( int x = 0; x <= 3; x++ ) {
        pcl::PointXYZ p;
        p.x = x;
        p.y = y2;
        pcl.push_back(p);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr = pcl.makeShared();

    vector<pcl::PointCloud<pcl::PointXYZ>> clusters = dbscan.findClusters(pclPtr);
    EXPECT_EQ(2, clusters.size());
    EXPECT_EQ(4, clusters[0].size());
    EXPECT_EQ(4, clusters[1].size());
}

TEST(DBSCAN, TestClusterTwoSlopedLines){
    int min_neighbours = 2;
    int radius = 5;
    DBSCAN dbscan(min_neighbours, radius);

    pcl::PointCloud<pcl::PointXYZ> pcl;

//    first line
    int y1 = 100;
    int m1 = 1;
    int firstLineLength = 100;
    for( int x = 0; x < firstLineLength; x++ ) {
        pcl::PointXYZ p;
        p.x = x;
        p.y = m1*x+y1;
        pcl.push_back(p);
    }
//second line
    int y2 = -100;
    int m2 = 1;
    int secondLineLength = 100;
    for( int x = 0; x < secondLineLength; x++ ) {
        pcl::PointXYZ p;
        p.x = x;
        p.y = m2*x+y2;
        pcl.push_back(p);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr = pcl.makeShared();

    vector<pcl::PointCloud<pcl::PointXYZ>> clusters = dbscan.findClusters(pclPtr);
    EXPECT_EQ(2, clusters.size());
    EXPECT_EQ(firstLineLength, clusters[0].size());
    EXPECT_EQ(secondLineLength, clusters[1].size());
}

TEST(DBSCAN, TestClusterTwoSlopedLinesWithOutliers){
    int min_neighbours = 2;
    int radius = 5;
    DBSCAN dbscan(min_neighbours, radius);

    pcl::PointCloud<pcl::PointXYZ> pcl;

//    first line
    int y1 = 100;
    int m1 = 1;
    int firstLineLength = 100;
    for( int x = 0; x < firstLineLength; x++ ) {
        pcl::PointXYZ p;
        p.x = x;
        p.y = m1*x+y1;
        pcl.push_back(p);
    }
//second line
    int y2 = -100;
    int m2 = 1;
    int secondLineLength = 100;
    for( int x = 0; x < secondLineLength; x++ ) {
        pcl::PointXYZ p;
        p.x = x;
        p.y = m2*x+y2;
        pcl.push_back(p);
    }

//    outliers
    pcl.push_back(pcl::PointXYZ(999999,999999,0));
    pcl.push_back(pcl::PointXYZ(-999999,-999999,0));

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr = pcl.makeShared();

    vector<pcl::PointCloud<pcl::PointXYZ>> clusters = dbscan.findClusters(pclPtr);
    EXPECT_EQ(2, clusters.size());
    EXPECT_EQ(firstLineLength, clusters[0].size());
    EXPECT_EQ(secondLineLength, clusters[1].size());
}

TEST(DBSCAN, TestClusterTwoLongHorizontalLines){
    int min_neighbours = 1;
    int radius = 5;
    DBSCAN dbscan(min_neighbours, radius);

    pcl::PointCloud<pcl::PointXYZ> pcl;

//    first line
    int y1 = 3;
    int firstLineLength = 500;
    int xinit = -10;
    for( int x = xinit; x < xinit+firstLineLength; x++ ) {
        pcl::PointXYZ p;
        p.x = x;
        p.y = y1;
        pcl.push_back(p);
    }
//second line
    int y2 = -3;
    int secondLineLength = 500;
    for( int x = xinit; x < xinit+secondLineLength; x++ ) {
        pcl::PointXYZ p;
        p.x = x;
        p.y = y2;
        pcl.push_back(p);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr = pcl.makeShared();

    std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters = dbscan.findClusters(pclPtr);
    EXPECT_EQ(2, clusters.size());
    EXPECT_EQ(firstLineLength, clusters[0].size());
    EXPECT_EQ(secondLineLength, clusters[1].size());
}

TEST(DBSCAN, TestClusterBorder){
    int min_neighbours = 1;
    int radius = 6;
    DBSCAN dbscan(min_neighbours, radius);

    pcl::PointCloud<pcl::PointXYZ> pcl;

//    first line
    float y1 = 3.000001;
    int firstLineLength = 3;
    int xinit = -10;
    for( int x = xinit; x < xinit+firstLineLength; x++ ) {
        pcl::PointXYZ p;
        p.x = x;
        p.y = y1;
        pcl.push_back(p);
    }
//second line
    float y2 = -3;
    int secondLineLength = 3;
    for( int x = xinit; x < xinit+secondLineLength; x++ ) {
        pcl::PointXYZ p;
        p.x = x;
        p.y = y2;
        pcl.push_back(p);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr = pcl.makeShared();

    vector<pcl::PointCloud<pcl::PointXYZ>> clusters = dbscan.findClusters(pclPtr);
    EXPECT_EQ(2, clusters.size());
    EXPECT_EQ(firstLineLength, clusters[0].size());
    EXPECT_EQ(secondLineLength, clusters[1].size());
}

TEST(DBSCAN, TestClusterTwoPolynomialLines){
    int min_neighbours = 1;
    int radius = 3;
    DBSCAN dbscan(min_neighbours, radius);

    pcl::PointCloud<pcl::PointXYZ> pcl;

//    first line
    int y1 = 10;
    int firstLineLength = 40;
    int xinit = -20;
    float a = 0.002;
    for( int x = xinit; x < xinit+firstLineLength; x++ ) {
        pcl::PointXYZ p;
        p.x = x;
        p.y = a*pow(x,3)+y1;
        pcl.push_back(p);
    }
//second line
    int y2 = -10;
    int secondLineLength = 40;
    for( int x = xinit; x < xinit+secondLineLength; x++ ) {
        pcl::PointXYZ p;
        p.x = x;
        p.y = a*pow(x,3)+y2;
        pcl.push_back(p);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr = pcl.makeShared();

    vector<pcl::PointCloud<pcl::PointXYZ>> clusters = dbscan.findClusters(pclPtr);
    EXPECT_EQ(2, clusters.size());
    EXPECT_EQ(firstLineLength, clusters[0].size());
    EXPECT_EQ(secondLineLength, clusters[1].size());
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}