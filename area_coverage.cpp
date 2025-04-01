#include "area_coverage.h"


Area_Coverage::Area_Coverage(int rows, int cols)
    : rows(rows), cols(cols)

{
  map = std::vector<std::vector<int>>(rows, std::vector<int>(cols, 0));
  pheromone = std::vector<std::vector<int>>(rows, std::vector<int>(cols, 0));
}

struct Node
{
  int row, col;
  int g, h;
  Node *parent;

  Node(int r, int c, int g_val, int h_val, Node *p) : row(r), col(c), g(g_val), h(h_val), parent(p)
  {
  }

  int get_f() const
  {
    return g + h;
  }
};
// Define map dimensions

// Define movement directions (right, down, left, up)
const int directions_row[] = {0, 1, 0, -1};
const int directions_col[] = {1, 0, -1, 0};

// Define comparison function for priority queue
struct Area_Coverage::CompareNodes
{
  bool operator()(const Node *a, const Node *b) const
  {
    return a->get_f() > b->get_f();
  }
};



// fill map with zeros for empty
void Area_Coverage::set_map_zeros()
{
  for (int i = 0; i < map.size(); i++)
  {
    for (int j = 0; j < map[i].size(); j++)
    {
      map[i][j] = 0;
    }
  }
}

bool Area_Coverage::area_done()
{
  for (int i = 0; i < rows; ++i)
  {
    for (int j = 0; j < cols; ++j)
    {
      if (pheromone[i][j] == 0)
      {
        return false;
      }
    }
  }
  return true;
}

// Function to check if a Cell is valid and in bounds of map
bool Area_Coverage::is_valid(int row, int col)
{
  return row >= 0 && row < rows && col >= 0 && col < cols;
}

// Heuristic function to calculate Manhattan distance with obstacle penalty
double Area_Coverage::get_heuristic(Area_Coverage::Cell current,
                                    Area_Coverage::Cell goal)
{
  double distance = abs(current.row - goal.row) + abs(current.col - goal.col);

  // Penalize visiting obstacles
  if (map[current.row][current.col] == 1)
  {
    distance += 50;
  }
  if (pheromone[current.row][current.col] > 3)
  {
    distance += 20;
  }
  return distance;
}

// Function to generate waypoints in zigzag pattern
void Area_Coverage::get_waypoints(std::vector<Area_Coverage::Cell> &waypoints)
{
  for (int row = 0; row < map.size(); row++)
  {
    if (row % 2 == 0)
    {
      // if even row, get first and then last column waypoint
      for (int col = 0; col < map[row].size(); col++)
      {
        if (map[row][col] == 0)
        {
          if (col == 0 || col == map[row].size() - 1)
          {
            waypoints.push_back({row, col});
          }
        }
        else
        {
          // if row has an object, get way points to left and right of
          // object if it is empty
          if (map[row][col - 1] == 0 && col - 1 >= 0)
          {
            waypoints.push_back({row, col - 1});
          }
          if (map[row][col + 1] == 0 && col + 1 < cols)
          {
            waypoints.push_back({row, col + 1});
          }
        }
      }
    }
    else
    {
      // right to left
      for (int col = cols - 1; col >= 0; --col)
      {
        // get last column and first column waypoints
        if (map[row][col] == 0)
        {
          if (col == cols - 1 || col == 0)
          {
            waypoints.push_back({row, col});
          }
        }
        else
        {
          // get waypoints left and right of object if empty

          if (map[row][col + 1] == 0 && col + 1 < cols)
          {
            waypoints.push_back({row, col + 1});
          }
          if (map[row][col - 1] == 0 && col - 1 >= 0)
          {
            waypoints.push_back({row, col - 1});
          }
        }
      }
    }
  }
}

bool Area_Coverage::check_goal(Area_Coverage::Cell &goal)
{
  {
    // Check if the goal is within the bounds of the map
    if (goal.row < 0 || goal.row >= map.size() || goal.col < 0 || goal.col >= map[0].size())
    {
      return false; 
    }

    // Check for goal's immediate surroundings
    if (goal.col == 0 || goal.col == map[0].size() - 1) // If it's in the first or last column
    {
      if (goal.row + 1 < map.size() && map[goal.row + 1][goal.col] == 1 &&       
          (map[goal.row][goal.col - 1] == 1 || map[goal.row][goal.col + 1] == 1)) 
      {
        return false;
      }
    }
    else if (goal.row + 1 < map.size() && goal.col - 1 >= 0 && goal.col + 1 < map[0].size())
    {
      if (map[goal.row + 1][goal.col] == 1 && // Down
          map[goal.row][goal.col - 1] == 1 && // Left
          map[goal.row][goal.col + 1] == 1 && // Right
          map[goal.row - 1][goal.col] == 1)   // Up
      {
        return false;
      }
    }
    else if (goal.row - 1 >= 0 && goal.col - 1 >= 0 && goal.col + 1 < map[0].size())
    {
      if ((map[goal.row][goal.col - 1] == 1 || 
        map[goal.row][goal.col + 1] == 1) && 
          map[goal.row - 1][goal.col] == 1)                                         
      {
        return false;
      }
    }

    return true;
  }
}
// Function to run A* algorithm
std::vector<Area_Coverage::Cell>
Area_Coverage::run_a_star(Area_Coverage::Cell &start,
                          Area_Coverage::Cell &goal)
{
  // Define priority queue for open set
  std::priority_queue<Node *, std::vector<Node *>, CompareNodes> open_set;

  if (!check_goal(goal))
  {
    return std::vector<Area_Coverage::Cell>();
  }
  // Initialize start Node
  Node *startNode = new Node(start.row, start.col, 0, 0, nullptr);
  open_set.push(startNode);

  // Define closed set to track explored nodes
  std::vector<std::vector<bool>> closed(rows, std::vector<bool>(cols, false));
  while (!open_set.empty())
  {
    // Get Node with lowest score from open set
    Node *current = open_set.top();
    open_set.pop();

    // Check if current Node is the goal
    if (current->row == goal.row && current->col == goal.col)
    {
      // get path
      std::vector<Area_Coverage::Cell> path;
      while (current != nullptr)
      {
        path.push_back({current->row, current->col});
        pheromone[current->row][current->col] =
            (pheromone[current->row][current->col] + 1);
        current = current->parent;
      }
      // reverse path
      reverse(path.begin(), path.end());
      return path;
    }

    // Mark current Node as closed
    closed[current->row][current->col] = true;

    // explore neighbors
    for (int i = 0; i < 4; ++i)
    {
      int new_row = current->row + directions_row[i];
      int new_col = current->col + directions_col[i];

      // Check if neighbor is valid, unexplored, and not an object
      if (is_valid(new_row, new_col) && !closed[new_row][new_col] && map[new_row][new_col] != 1)
      {
        // Calculate neighbor's g and h values
        int new_g = current->g + 1;
        int new_h = get_heuristic({new_row, new_col}, goal);

        // Create neighbor Node
        Node *neighbor = new Node(new_row, new_col, new_g, new_h, current);

        // Add neighbor to open set
        open_set.push(neighbor);
      }
    }
  }
  // no path
  std::cout << "No path found\n";
  return std::vector<Area_Coverage::Cell>();
}

// Function to mark path on the map
void Area_Coverage::mark_path(std::vector<Area_Coverage::Cell> &path)
{
  for (int i = 0; i < path.size(); i++)
  {
    map[path[i].row][path[i].col] = 2;
  }
}
void Area_Coverage::display_pheremone()
{
  for (int i = 0; i < pheromone.size(); ++i)
  {
    for (int j = 0; j < cols; ++j)
    {
      std::cout << pheromone[i][j] << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

// Display the map with final path
void Area_Coverage::display_map()
{
  std::cout << "Map:" << std::endl;
  for (int i = 0; i < rows; ++i)
  {
    for (int j = 0; j < cols; ++j)
    {
      if (map[i][j] == 1)
      {
        std::cout << "1 "; // Obstacle
      }
      else if (map[i][j] == 2)
      {
        std::cout << "* "; // Path
      }
      else
      {
        std::cout << "0 "; // Free space
      }
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}
// A* algorithm modified for zigzag coverage
std::vector<Area_Coverage::Cell>
Area_Coverage::area_coverage(std::vector<Area_Coverage::Cell> &waypoints,
                             Area_Coverage::Cell &start,
                             Area_Coverage::Cell &goal)
{
  std::vector<Area_Coverage::Cell> path;
  Area_Coverage::Cell current;
  //Check if object is at start
  if (map[start.row][start.col]== 1){
    std::cout<< "Error: There is an object at start\n";
    return std::vector<Area_Coverage::Cell>();
  }

  for (int i = 1; i < waypoints.size(); ++i)
  {
    // condition to prevent going to unreachable waypoints
    if (i == 1)
    {
      current = waypoints[0];
    }
    else
    {
      current = path.back();
  
    }
    Area_Coverage::Cell next = waypoints[i];
   
    bool object = false;
    // Run A* algorithm between current and next waypoint
    std::vector<Area_Coverage::Cell> segment_path = run_a_star(current, next);
    
    path.insert(path.end(), segment_path.begin(), segment_path.end());
    mark_path(path);
    //show path movement 
   // display_map();
    //display_pheremone(pheromone);
  }

  return path;
}

void Area_Coverage::set_objects(int row, int col){
  map[row][col] = 1;
}