/*
 * Created By: Min Gyo Kim
 * Created On: January 20, 2018
 * Description: Ros tests for Line Extractor Node
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <mapping_igvc/LineObstacle.h>
#include <pcl_conversions/pcl_conversions.h>

/**
 * This is the helper class which will publish and subscribe messages which will test the node being instantiated
 * It contains at the minimum:
 *      publisher - publishes the input to the node
 *      subscriber - publishes the output of the node
 *      callback function - the callback function which corresponds to the subscriber
 *      getter function - to provide a way for gtest to check for equality of the message recieved
 */
class LineExtractorRosTest : public testing::Test{
protected:
    virtual void SetUp(){
//        nh_.setParam("line_extractor/degree_polynomial", 3);
//        nh_.setParam("line_extractor/lambda", 0);
//        nh_.setParam("line_extractor/min_neighbours", 1);
//        nh_.setParam("line_extractor/radius", 80);

        test_publisher = nh_.advertise<sensor_msgs::PointCloud2>("/pcl", 1);
        test_subscriber = nh_.subscribe("/lines", 1, &LineExtractorRosTest::callback, this);

        // Let the publishers and subscribers set itself up timely
        ros::Rate loop_rate(1);
        loop_rate.sleep();
    }

    ros::NodeHandle nh_;
    mapping_igvc::LineObstacle lineObstacle;
    ros::Publisher test_publisher;
    ros::Subscriber test_subscriber;

public:

    void callback(const mapping_igvc::LineObstacle& line){
        lineObstacle = line;
    }
};

TEST_F(LineExtractorRosTest, getAngularVel){


    pcl::PointCloud<pcl::PointXYZ> pcl;

    int y1 = 1000;
    float m1 = 7;
    float m2 = -0.7;
    float m3 = 0.007;

    int num_points = 100;
    for( float x = 0; x < num_points; x ++ ) {
        float true_y = m1*x+m2*pow(x,2)+m3*pow(x,3)+y1;
        float true_x = x;

        float noise_y = ((float) rand() / (RAND_MAX))*2-1;
        float noise_x = ((float) rand() / (RAND_MAX))*2-1;

        float deformed_y = true_y + noise_y;
        float deformed_x = true_x + noise_x;

        pcl::PointXYZ p;
        p.x = deformed_x;
        p.y = deformed_y;
        pcl.push_back(p);
    }

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(pcl,msg);

    test_publisher.publish(msg);
    ros::Rate loop_rate(1);

    // Wait for the message to get passed around
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious: http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    ASSERT_EQ(lineObstacle.coefficients.size(), 4);
    EXPECT_NEAR(lineObstacle.coefficients[0], y1, 10);
    EXPECT_NEAR(lineObstacle.coefficients[1], m1, 5);
    EXPECT_NEAR(lineObstacle.coefficients[2], m2, 5);
    EXPECT_NEAR(lineObstacle.coefficients[3], m3, 5);
    EXPECT_NEAR(lineObstacle.x_min, 0, 1);
    EXPECT_NEAR(lineObstacle.x_max, 99, 1);
}


int main(int argc, char **argv) {
    // !! Don't forget to initialize ROS, since this is a test within the ros framework !!
    ros::init(argc, argv, "line_extractor_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
