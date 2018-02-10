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

    /*
     * Linear Equation to solve:
     * X' * X + lambda * I = X' * y
     *
     * X is a matrix of size (n,d) where n is the number of points in the cluster,
     * and d is the polynomial degree + 1. (+1 for linear bias)
     * Each row of X represents the x coordinate of a point.
     * Given x for the x coordinate of a point and column for the column index
     * in the matrix X:
     * The first column of X is 1 for linear bias, and the rest of the columns
     * contain the value of pow(x, column-1).
     *
     * lambda is the regularization parameter. The higher this number,
     * The higher the penalty on the size of the coefficients of the mathematical lines.
     *
     * y is a column vector of size (n), where n is the number of points in the cluster.
     * Each row of the vector corresponds to the y coordinate of a point.
     */

    MatrixXf X(n, polyDegree+1);
    VectorXf y(n);


    for( unsigned int i = 0; i < cluster.size(); i++ ) {
        PointXYZ point = cluster[i];

        VectorXf row(polyDegree+1);
        row(0) = 1;

        for( unsigned int j = 1; j < polyDegree+1; j++ ) {
            row(j) = pow( point.x, j );
        }

        X.row(i) = row;

        y(i) = point.y;
    }

    MatrixXf regularization(polyDegree+1, polyDegree+1);
    regularization.setIdentity();
    regularization *= lambda;

    MatrixXf left(X.transpose()*X + regularization);
    MatrixXf right(X.transpose()*y);

    VectorXf line(n);
    line = (left).ldlt().solve(right);

    return line;
}

