## Project Overview:
During my internship, I collaborated with a team to develop an autonomous robot designed to be equipped with a pressure washer system for cleaning pool decks. Our primary challenge was to design a navigation system that could effectively maneuver around obstacles while ensuring complete coverage of the area.

## Key Contributions:

Area Coverage Algorithm: I developed an area coverage algorithm in C++ using the A* Search method to enable efficient robot navigation, obstacle avoidance, and cover a defined area. The algorithm was integrated with Madara Middlware and utilized sensor data from Mid-360 LiDAR and Emlid GNSS. 

LiDAR Integration for Object Detection: I utilized LiDAR to capture point clouds and applied plane segmentation to identify flat surfaces. Using the PCL (Point Cloud Library), I extracted Euclidean clusters to group objects and determine their locations on a 2D grid map. This approach enabled effective object detection and localization for the autonomous robot.


<img width="938" height="396" alt="343584082-76114255-423c-47a5-8d30-e293fb3a21ec" src="https://github.com/user-attachments/assets/3b0f339a-2c8e-45e7-9d89-56252b3f2b6c" />


GPS Waypoint Generation: We integrated a GPS system using the Emlid Reach RS3 to accurately capture the latitude and longitude of objects. The receivers tracked the robot's location, which allowed me to convert the LiDAR Cartesian coordinates into GPS coordinates. This data was then combined with the area coverage algorithm to enable precise waypoint navigation.

## Demo Video
[Area Coverage Around Obstacles Demo ](https://youtu.be/kLdjPo3Pac8)

## Tools:
Mid-360 (LiDAR): LiDAR sensor for high-precision 3D mapping.
Emlid Reach RS3: GNSS receiver for accurate positioning.
Reach M2 LoRa Radios: Long-range communication system for data transmission.
NVIDIA Jetson Orin: AI computing platform for real-time processing and analysis.
Programming Language: C++
Cmake: Used for building and testing our project.
Madara Middleware (madara.ai): Middleware for multi-agent systems and autonomous applications.
