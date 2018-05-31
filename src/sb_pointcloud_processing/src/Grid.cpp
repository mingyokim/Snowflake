//
// Created by min on 27/05/18.
//

#include <Grid.h>

using namespace LineExtractor;

Grid::Grid(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr, float radius, PointCloudUtils::XYRange xy_range) {
    this->_xy_range = xy_range;
    this->_radius = radius;

    this->_rows = std::ceil((xy_range.y.max - xy_range.y.min)/radius);
    this->_cols = std::ceil((xy_range.x.max - xy_range.x.min)/radius);

    this->_top_left_point = pcl::PointXYZ(xy_range.x.min, xy_range.y.max, 0.0);

    this->_grid = std::vector<std::vector<Cell>>(this->_rows, std::vector<Cell>(this->_cols, Cell()));

    populateGrid(pcl_ptr);
}

std::vector<pcl::PointXYZ> Grid::getPotentialNeighboursOfPoint(pcl::PointXYZ point) {
    // TODO
    return std::vector<pcl::PointXYZ>();
}

void Grid::populateGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr) {
    pcl::PointCloud<pcl::PointXYZ> pcl = *pcl_ptr;

    for (unsigned int i = 0; i < pcl.size(); i++ ) {
        addPointToACell(pcl[i]);
    }
}

void Grid::addPointToACell(pcl::PointXYZ point) {
    Cell *cell = getCellOfPoint(point);
    cell->push_back(point);
}

Cell* Grid::getCellOfPoint(pcl::PointXYZ point) {
    int row = getCellRowOfPoint(point);
    int col = getCellColOfPoint(point);

    return &this->_grid[row][col];
}

int Grid::getCellRowOfPoint(pcl::PointXYZ point) {
    return (this->_top_left_point.y - point.y) / this->_radius;
}

int Grid::getCellColOfPoint(pcl::PointXYZ point) {
    return (point.x - this->_top_left_point.x) / this->_radius;
}
