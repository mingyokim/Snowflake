/*
 * Created By: Min Gyo Kim
 * Created On: February 3, 2018
 * Description: Calculates line of best fit for each cluster of points
 */

#include <Regression.h>

vector<VectorXf> Regression::getLinesOfBestFit(vector<PointCloud<PointXYZ>> clusters, unsigned int polyDegree, float lambda) {
    vector<VectorXf> lines;

    for( unsigned int i = 0; i < clusters.size(); i++ ) {
        lines.push_back(getLineOfCluster(clusters[i], polyDegree, lambda));
    }

    return lines;
}

VectorXf Regression::getLineOfCluster(PointCloud<PointXYZ> cluster, unsigned int polyDegree, float lambda) {
    unsigned int n = cluster.size();
    MatrixXf mat(n, polyDegree+1);
    VectorXf y(n);

    for( unsigned int i = 0; i < cluster.size(); i++ ) {
        PointXYZ point = cluster[i];

        VectorXf row(polyDegree+1);
        row(0) = 1;

        for( unsigned int j = 1; j < polyDegree+1; j++ ) {
            row(j) = pow( point.x, j );
        }

        mat.row(i) = row;

        y(i) = point.y;
    }

    MatrixXf regularization(polyDegree+1, polyDegree+1);
    regularization.setIdentity();
    regularization *= lambda;

    MatrixXf left(mat.transpose()*mat + regularization);
    MatrixXf right(mat.transpose()*y);

    VectorXf line(n);
    line = (left).ldlt().solve(right);

    return line;
}

