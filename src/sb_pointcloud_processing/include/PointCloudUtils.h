//
// Created by min on 27/05/18.
//

#ifndef PROJECT_POINTCLOUDUTILS_H
#define PROJECT_POINTCLOUDUTILS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudUtils {
public:
    struct Range {
        double x_min;
        double x_max;

        double y_min;
        double y_max;
    };

    static Range getRangeOfPcl(pcl::PointCloud<pcl::PointXYZ> pcl) {
        Range range;

        if (pcl.size()) {
            range.x_min = range.x_max = pcl[0].x;
            range.y_min = range.y_max = pcl[0].y;
        } else {
            range.x_min = range.x_max = -1;
            range.y_min = range.y_max = -1;
            return range;
        }

        for (unsigned int i = 0; i < pcl.size(); i++) {
            if (pcl[i].x < range.x_min) { range.x_min = pcl[i].x; }
            if (pcl[i].x > range.x_max) { range.x_max = pcl[i].x; }

            if (pcl[i].y < range.y_min) { range.y_min = pcl[i].y; }
            if (pcl[i].y > range.y_max) { range.y_max = pcl[i].y; }
        }
    }
};

#endif //PROJECT_POINTCLOUDUTILS_H
