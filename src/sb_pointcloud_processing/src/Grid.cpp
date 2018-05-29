//
// Created by min on 27/05/18.
//

#include <Grid.h>

using namespace LineExtractor;

Grid::Grid(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr, float radius, PointCloudUtils::XYRange xy_range) {
    this->_xy_range = xy_range;

    this->_rows = std::ceil((xy_range.y.max - xy_range.y.min)/radius);
    this->_cols = std::ceil((xy_range.x.max - xy_range.x.min)/radius);

    this->_top_left_point = pcl::PointXYZ(xy_range.x.min, xy_range.y.max, 0.0);

    this->_grid = std::vector<std::vector<Cell>>(this->_rows, std::vector<Cell>(this->_cols, Cell()));
}
