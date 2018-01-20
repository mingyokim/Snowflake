/*
 * Created By: Min Gyo Kim
 * Created On: January 20, 2018
 * Description: Node implementation for Line Extractor Node
 */

#include <LineExtractorNode.h>

LineExtractorNode::LineExtractorNode(int argc, char **argv, std::string node_name) {
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    //TODO: change subscriber topic name when it's determined
    std::string topic_to_subscribe_to = "/pcl"; // dummy topic name
    int refresh_rate = 10;
    subscriber = nh.subscribe(topic_to_subscribe_to, refresh_rate, &LineExtractorNode::pclCallBack,this);

    //TODO: change publisher topic name when it's determined
    std::string topic_to_publish_to = "/lines"; //dummy topic name
    uint32_t queue_size = 1;
    publisher = nh.advertise<std_msgs::Float32>(topic_to_publish_to, queue_size);
}

void LineExtractorNode::pclCallBack(sensor_msgs::PointCloud pointCloud) {
    this->points = new std::vector(pointCloud.points);
    extractLines();
    return;
}

void LineExtractorNode::extractLines() {
//TODO: call line extractor to actually extract lines

    //dummy message type and data
    std_msgs::Float32 msg;
    msg.data = 9.9;
    publisher.publish(msg);

    return;
}