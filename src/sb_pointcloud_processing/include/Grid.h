//
// Created by min on 27/05/18.
//

#ifndef PROJECT_GRID_H
#define PROJECT_GRID_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace LineExtractor {
    typedef std::vector<pcl::PointXYZ> Cell;

    class Grid {
    public:
        Grid(pcl::PointCloud<pcl::PointXYZ> pcl, float radius);

        std::vector<pcl::PointXYZ> getPotentialNeighboursOfPoint(pcl::PointXYZ point);

    private:
        void populateGrid(pcl::PointCloud<pcl::PointXYZ> pcl);
        void addPointToCell(pcl::PointXYZ point);
        Cell getCellOfPoint(pcl::PointXYZ point);
        int getCellRowOfPoint(pcl::PointXYZ point);
        int getCellColOfPoint(pcl::PointXYZ point);

        std::vector<Cell> getAdjacentCellsOfPoint(pcl::PointXYZ point);
    };
}

#endif //PROJECT_GRID_H
