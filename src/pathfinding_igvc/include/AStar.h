/*
 * Created By: Min Gyo Kim
 * Created On: May 9th 2018
 * Description: Class that implements A* algorithm to find the shortest path
 * between two points given an occupancy grid. Basically copies the algorithm from
 * https://www.geeksforgeeks.org/a-search-algorithm/
 */

#ifndef PATHFINDING_IGVC_ASTAR_H
#define PATHFINDING_IGVC_ASTAR_H

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <stack>

class AStar {
  public:
    /*
     * Assumes the value at a cell of occupancy grid is either GRID_FREE
     * or GRID_OCCUPIED
     */
    static const int GRID_FREE     = 0;
    static const int GRID_OCCUPIED = 100;

    /*
     * A representation of a point in grid with its col and row
     */
    struct GridPoint {
        int col;
        int row;
        GridPoint(int r = 0, int c = 0) : col(c), row(r) {};
    };

    /*
     * A representation of a point in grid along with its score.
     * Its score, f, is equal to g+h, where:
     * g is "the movement cost from the starting point" to this point
     * h is "the estimated movement cost to move from this point to
     * the final destination".
     */
    typedef std::pair<double, std::pair<int, int>> GridPointWithScore;

    /*
     * A structure that holds the necessary details for a cell
     * It stores its parent as well as its score parameters,
     * f, g, and h.
     */
    struct CellDetail
    {
        // parent of cell
        GridPoint parent;
        // f = g + h
        double f = FLT_MAX, g = FLT_MAX, h = FLT_MAX;
    };

    /*
     * parameters for storing the size of the grid,
     * obtained from the occupancy grid metadata
     */
    int _num_rows;
    int _num_cols;

    /*
     * parameters for storing the starting and goal
     * points in the grid
     */
    GridPoint _start;
    GridPoint _goal;

    /*
     * stores the grid, accessible to all functions
     * of this class
     */
    std::vector<int8_t> _grid;

    /*
     * _cell_details stores the CellDetail for every
     * cell in the grid
     */
    CellDetail **_cell_details;

    /*
     * _closed_list stores whether or not a cell has
     * already been visited for every cell in the grid
     */
    bool **_closed_list;

    /*
     * _open_list stores which cells to visit
     */
    std::set<GridPointWithScore> _open_list;

    /**
     * Takes an occupancy grid as well as start and goal points, and calculates
     * the shortest path from start to goal.
     *
     * @param occupancy_grid occupancy grid
     * @param start GridPoint containing row and column of the starting cell
     * @param goal GridPoint containing row and column of the goal cell
     * @return points stacked in order, where the top contains the starting
     * GridPoint and the bottom contains the goal GridPoint
     */
    static std::stack<GridPoint> run(nav_msgs::OccupancyGrid occupancy_grid,
                              GridPoint start,
                              GridPoint goal);

private:
    /**
     * Initializes the class with the given occupancy_grid and start and
     * goal grid points
     *
     * The constructor is private because a path would only be found once
     * for a given occupancy grid and start and goal points.
     *
     * @param occupancy_grid occupancy grid
     * @param start
     * @param goal
     */
    AStar(nav_msgs::OccupancyGrid occupancy_grid, GridPoint start,
          GridPoint goal);

    /**
     * Performs A* search and returns the path in a stack.
     * The occupancy grid as well as start and goal points are accessed
     * through the class' instance variables.
     *
     * @return
     */
    std::stack<GridPoint> aStarSearch();

    bool isValid(AStar::GridPoint point);
    bool isUnBlocked(AStar::GridPoint point);
    bool isDestination(AStar::GridPoint point);
    double calculateHValue(AStar::GridPoint point);
    std::stack<GridPoint> tracePath();
    bool processSuccessor(GridPoint successor, GridPoint parent);
};

#endif // PATHFINDING_IGVC_ASTAR_H
