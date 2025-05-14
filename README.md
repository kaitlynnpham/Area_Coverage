# Area_Coverage
This is a simplified area coverage algorithm in C++ for autonomous navigation around obstacles. 

This algorithm was originally combined with Mid-360 LiDAR to create a 2D obstacle map and Emlid GNSS to convert the cartesian waypoints into georeferenced GPS coordinates(latitude/longitude). 

The use of A* (A-Star) for generating the shortest path between boundary waypoints was inspired by academic research presented in the following article: https://www.mdpi.com/1424-8220/18/8/2585 

