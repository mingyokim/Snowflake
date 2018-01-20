/*
 * Created By: Min Gyo Kim
 * Created On: January 20, 2018
 * Description: Ros tests for Line Extractor Node
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud.h>

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
        test_publisher = nh_.advertise<sensor_msgs::PointCloud>("/pcl", 1);
        test_subscriber = nh_.subscribe("/lines", 1, &LineExtractorRosTest::callback, this);

        // Let the publishers and subscribers set itself up timely
        ros::Rate loop_rate(1);
        loop_rate.sleep();
    }

    ros::NodeHandle nh_;
    float message_output;
    ros::Publisher test_publisher;
    ros::Subscriber test_subscriber;

public:

    void callback(const std_msgs::Float32& scan){
        message_output = scan.data;
    }
};

TEST_F(LineExtractorRosTest, getAngularVel){
    sensor_msgs::PointCloud pcl;

    test_publisher.publish(pcl);
    ros::Rate loop_rate(1);

    // Wait for the message to get passed around
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious: http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    EXPECT_FLOAT_EQ(9.9, message_output);
}


int main(int argc, char **argv) {
    // !! Don't forget to initialize ROS, since this is a test within the ros framework !!
    ros::init(argc, argv, "line_extractor_node_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
