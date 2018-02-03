/*
 * Created By: Min Gyo Kim
 * Created On: February 3, 2018
 * Description: Tests calculation of line of best fit
 */

#include <gtest/gtest.h>
#include <Regression.h>

TEST(Regression, OnePerfectLinearFit) {
    unsigned int polyDegree = 1;

    pcl::PointCloud<pcl::PointXYZ> pcl;

    int y1 = 100;
    int m1 = 1;
    int firstLineLength = 100;
    for( int x = 0; x < firstLineLength; x++ ) {
        pcl::PointXYZ p;
        p.x = x;
        p.y = m1*x+y1;
        pcl.push_back(p);
    }

    vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
    clusters.push_back(pcl);

    vector<VectorXf> lines = Regression::getLinesOfBestFit(clusters, polyDegree);

    EXPECT_FLOAT_EQ(lines[0](0), y1);
    EXPECT_FLOAT_EQ(lines[0](1), m1);
}

TEST(Regression, MultiplePerfectLinearFits) {
    unsigned int polyDegree = 1;

    unsigned int num_lines = 3;
    vector<float> y_ints = {5, 10, -99};
    vector<float> slopes = {-2, 7, -10};
    unsigned int num_points = 1000;

    vector<pcl::PointCloud<pcl::PointXYZ>> clusters;

    for( unsigned int i = 0; i < num_lines; i++ ) {
        pcl::PointCloud<pcl::PointXYZ> pcl;
        for( float x = 0; x < num_points; x += 0.1 ) {
            float y = y_ints[i];
            y += slopes[i]*x;
            pcl.push_back(PointXYZ(x,y,0));
        }
        clusters.push_back(pcl);
    }

    vector<VectorXf> lines = Regression::getLinesOfBestFit(clusters, polyDegree, 10);

    for( unsigned int i = 0; i < num_lines; i++ ) {
        VectorXf line = lines[i];

        EXPECT_NEAR(line[0], y_ints[i], 1);
        EXPECT_NEAR(line[1], slopes[i], 1);
    }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}