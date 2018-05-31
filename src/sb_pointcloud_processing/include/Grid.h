//
// Created by min on 27/05/18.
//

#ifndef PROJECT_GRID_H
#define PROJECT_GRID_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <PointCloudUtils.h>

namespace LineExtractor {
    typedef std::vector<pcl::PointXYZ> Cell;

    class Grid {
    public:
        Grid(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr, float radius, PointCloudUtils::XYRange xy_range);

        std::vector<pcl::PointXYZ> getPotentialNeighboursOfPoint(pcl::PointXYZ point);

    private:
        PointCloudUtils::XYRange _xy_range;
        float _radius;

        int _rows;
        int _cols;

        pcl::PointXYZ _top_left_point;

        std::vector<std::vector<Cell>> _grid;

        void populateGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr);
        void addPointToACell(pcl::PointXYZ point);
        Cell* getCellOfPoint(pcl::PointXYZ point);
        int getCellRowOfPoint(pcl::PointXYZ point);
        int getCellColOfPoint(pcl::PointXYZ point);

        std::vector<Cell> getAdjacentCellsOfPoint(pcl::PointXYZ point);
    };
}

#endif //PROJECT_GRID_H
