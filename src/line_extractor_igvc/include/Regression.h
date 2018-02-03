/*
 * Created By: Min Gyo Kim
 * Created On: January 20, 2018
 * Description: Class declaration for Regression, which calculates
 *              line of best fit for each cluster of points
 */

#ifndef PROJECT_REGRESSION_H
#define PROJECT_REGRESSION_H

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace Eigen;
using namespace std;
using namespace pcl;

class Regression {
public:
    static vector<VectorXf> getLinesOfBestFit(vector<PointCloud<PointXYZ>> clusters, unsigned int polyDegree, float lambda=0);
private:
    static VectorXf getLineOfCluster(PointCloud<PointXYZ> cluster, unsigned int polyDegree, float lambda=0);
};


#endif //PROJECT_REGRESSION_H
