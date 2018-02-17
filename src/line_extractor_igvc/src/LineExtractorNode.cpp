/*
 * Created By: Min Gyo Kim
 * Created On: January 20, 2018
 * Description: Node implementation for Line Extractor Node
 */

#include <LineExtractorNode.h>

LineExtractorNode::LineExtractorNode(int argc,
                                     char** argv,
                                     std::string node_name) {
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string degree_polynomial_param = "degree_polynomial";
    int default_degree_polynomial       = 3;
    SB_getParam(private_nh,
                degree_polynomial_param,
                this->degreePoly,
                default_degree_polynomial);

    std::string lambda_param = "lambda";
    float default_lambda     = 0;
    SB_getParam(private_nh, lambda_param, this->lambda, default_lambda);

    std::string min_neighbours_param = "min_neighbours";
    int default_min_neighbours       = 3;
    SB_getParam(private_nh,
                min_neighbours_param,
                this->minNeighbours,
                default_min_neighbours);

    std::string radius_param = "radius";
    float default_radius     = 0.1;
    SB_getParam(private_nh, radius_param, this->radius, default_radius);

    if (areParamsInvalid()) { ros::shutdown(); }

    std::string topic_to_subscribe_to = "input_pointcloud"; // dummy topic name
    int refresh_rate                  = 10;
    subscriber                        = nh.subscribe(
    topic_to_subscribe_to, refresh_rate, &LineExtractorNode::pclCallBack, this);

    std::string topic_to_publish_to =
    "output_line_obstacle"; // dummy topic name
    uint32_t queue_size = 1;
    publisher           = private_nh.advertise<mapping_igvc::LineObstacle>(
    topic_to_publish_to, queue_size);

    this->dbscan.setRadius(this->radius);
    this->dbscan.setMinNeighbours(this->minNeighbours);
}

void LineExtractorNode::pclCallBack(
const sensor_msgs::PointCloud2ConstPtr input) {
    pcl::PCLPointCloud2 pcl_pc2;

    // convert sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
    pcl_conversions::toPCL(*input, pcl_pc2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(
    new pcl::PointCloud<pcl::PointXYZ>);

    // convert pcl::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

    // store converted pointcloud for use
    this->pclPtr = temp_cloud;

    // extract lines from the pointcloud
    extractLines();

    return;
}

void LineExtractorNode::extractLines() {
    this->clusters = this->dbscan.findClusters(this->pclPtr);
    std::vector<Eigen::VectorXf> lines =
    regression.getLinesOfBestFit(this->clusters, this->degreePoly);

    std::vector<mapping_igvc::LineObstacle> lineObstacles =
    vectorsToMsgs(lines);

    for (unsigned int i = 0; i < lineObstacles.size(); i++) {
        publisher.publish(lineObstacles[i]);
    }

    return;
}

bool LineExtractorNode::areParamsInvalid() {
    return this->degreePoly < 0 || this->lambda < 0 ||
           this->minNeighbours < 0 || this->radius < 0;
}

std::vector<mapping_igvc::LineObstacle>
LineExtractorNode::vectorsToMsgs(std::vector<Eigen::VectorXf> vectors) {
    std::vector<mapping_igvc::LineObstacle> msgs;

    for (unsigned int i = 0; i < vectors.size(); i++) {
        msgs.push_back(vectorToLineObstacle(vectors[i], i));
    }

    return msgs;
}

mapping_igvc::LineObstacle
LineExtractorNode::vectorToLineObstacle(Eigen::VectorXf v,
                                        unsigned int clusterIndex) {
    mapping_igvc::LineObstacle lineObstacle = mapping_igvc::LineObstacle();

    for (unsigned int i = 0; i < v.size(); i++) {
        lineObstacle.coefficients.push_back(v(i));
    }

    getClusterXRange(lineObstacle.x_min, lineObstacle.x_max, clusterIndex);

    return lineObstacle;
}

void LineExtractorNode::getClusterXRange(double& xmin,
                                         double& xmax,
                                         unsigned int clusterIndex) {
    pcl::PointCloud<pcl::PointXYZ> cluster = this->clusters[clusterIndex];

    double min, max;

    if (cluster.size()) {
        min = max = cluster[0].x;
    } else {
        xmin = xmax = -1;
        return;
    }

    for (unsigned int i = 0; i < cluster.size(); i++) {
        if (cluster[i].x < min) { min = cluster[i].x; }
        if (cluster[i].x > max) { max = cluster[i].x; }
    }

    xmin = min;
    xmax = max;
}