#include "area_coverage.h"
using namespace std; 


int main(){

 
    std::vector<Area_Coverage::Cell> waypoints;
     // starting point at top-left
    Area_Coverage::Cell start = {0, 0}; 
    Area_Coverage::Cell goal;

//Test Case #1
    Area_Coverage test1(10, 7); 

    //set objects

    test1.set_objects(2,4);
    test1.set_objects(3,6);
    test1.get_waypoints(waypoints);
    
    std::cout<< "Map before area coverage:\n";
    test1.display_map();
    goal = {waypoints[waypoints.size()-1].row, waypoints[waypoints.size()-1].col}; 
    std::vector<Area_Coverage::Cell> path = test1.area_coverage(waypoints, start, goal); 
    std::cout<< "Map after area coverage: \n";
    test1.display_map();


//Test Case #2

Area_Coverage test2(25, 20); 


//set objects

test2.set_objects(0,0);
test2.set_objects(5,15);
test2.get_waypoints(waypoints);

std::cout<< "Map before area coverage:\n";
test2.display_map();

goal = {waypoints[waypoints.size()-1].row, waypoints[waypoints.size()-1].col}; 

test2.area_coverage(waypoints, start, goal); 
std::cout<< "Map after area coverage: \n";
test2.display_map();


//Test Case #3

Area_Coverage test3(15, 10); 


//set objects

test3.set_objects(4,9);
test3.set_objects(5,14);
test3.set_objects(2,1);
test3.set_objects(9,3);
test3.set_objects(10,9);
test3.set_objects(13,7);

test3.get_waypoints(waypoints);

std::cout<< "Map before area coverage:\n";
test3.display_map();

goal = {waypoints[waypoints.size()-1].row, waypoints[waypoints.size()-1].col}; 

test3.area_coverage(waypoints, start, goal); 
std::cout<< "Map after area coverage: \n";
test3.display_map();

    return 0; 
}





