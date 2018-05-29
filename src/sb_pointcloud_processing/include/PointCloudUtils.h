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
        double min, max;
    } XRange, YRange;
    struct XYRange {
        Range x, y;
    };

    static XYRange getXYRangeOfPcl(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr) {
        XYRange range;
        range.x = getXRangeOfPcl(pcl_ptr);
        range.y = getYRangeOfPcl(pcl_ptr);
        return range;
    }

    static Range getXRangeOfPcl(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr) {
        pcl::PointCloud<pcl::PointXYZ> pcl = *pcl_ptr;
        Range x_range;

        if (pcl.size()) {
            x_range.min = x_range.max = pcl[0].x;
        } else {
            x_range.min = x_range.max = -1;
            return x_range;
        }

        for (unsigned int i = 0; i < pcl.size(); i++) {
            if (pcl[i].x < x_range.min) { x_range.min = pcl[i].x; }
            if (pcl[i].x > x_range.max) { x_range.max = pcl[i].x; }
        }
    }

    static Range getYRangeOfPcl(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr) {
        pcl::PointCloud<pcl::PointXYZ> pcl = *pcl_ptr;
        Range y_range;

        if (pcl.size()) {
            y_range.min = y_range.max = pcl[0].y;
        } else {
            y_range.min = y_range.max = -1;
            return y_range;
        }

        for (unsigned int i = 0; i < pcl.size(); i++) {
            if (pcl[i].y < y_range.min) { y_range.min = pcl[i].y; }
            if (pcl[i].y > y_range.max) { y_range.max = pcl[i].y; }
        }
    }
};

#endif //PROJECT_POINTCLOUDUTILS_H
