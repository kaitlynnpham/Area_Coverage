#include "area_coverage.h"
using namespace std; 


int main(){

    int rows =10;
    int cols =7; 
    Area_Coverage test1(rows, cols); 
     std::vector<std::vector<int>> pheromone(rows, std::vector<int>(cols, 0));
    std::vector<Area_Coverage::Cell> waypoints;
    

Area_Coverage::Cell start = {0, 0};  // starting point at top-left


//set objects

test1.set_objects(2,4);
test1.set_objects(3,6);






 test1.get_waypoints(waypoints);


Area_Coverage::Cell goal = {waypoints[waypoints.size()-1].row, waypoints[waypoints.size()-1].col}; 
std::vector< Area_Coverage::Cell > path = test1.area_coverage(pheromone,waypoints, start, goal); 


//  for (int i =0; i < path.size(); i++){
//     std::cout << path[i].row << " " << path[i].col << endl;
//  }


//area.display_pheremone(pheromone); 

    return 0; 
}





