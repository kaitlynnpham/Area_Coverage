#include <queue>
#include <vector>
#include "iostream"

#ifndef AREA_COVERAGE_H  
#define AREA_COVERAGE_H

/**
 * @brief Algorithm to cover an area by getting boundary waypoints and
 * waypoints around obstacles.
 *
 */

class Area_Coverage
{
private:
  int rows, cols;
  std::vector<std::vector<int>> map;
  std::vector<std::vector<int>> pheromone;

public:
  struct Cell
  {
    int row, col;
  };

  Area_Coverage(int rows, int cols);

  struct CompareNodes;

  void set_map_zeros();

  void set_objects(int row, int col);

  bool area_done();

  bool is_valid(int row, int col);

  double get_heuristic(Cell current,
                       Cell goal);

  void get_waypoints(std::vector<Cell> &waypoints);
  std::vector<Cell> run_a_star(Cell &start,
                               Cell &goal);

  void mark_path(std::vector<Cell> &path);

  void display_pheremone();

  bool check_goal( Cell &goal);

  void display_map();

  std::vector<Cell> area_coverage(std::vector<Cell> &waypoints, Cell &start,
                                  Cell &goal);

};

#endif 
