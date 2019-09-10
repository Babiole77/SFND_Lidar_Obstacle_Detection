# Sensor Fusion Self-Driving Car: LIDAR

<img src="https://github.com/Babiole77/SFND_Lidar_Obstacle_Detection/blob/master/media/ObstacleDetectionFPS.gif" width="700" height="400" />

The pipeline for LIDAR Obstacle Detection Algorithm consists of 3 main steps:
1. Point cloud segmentation using RANSAC algorithm. As a result the point cloud segmented into road and obstacles.
2. The resulting cloud of obstacles from the first step further clustered into different objects using Euclidean Clustering
3. Find bounding boxes for an each object


## Installation

### Linux Ubuntu 16

Install PCL, C++

The link here is very helpful, 
https://larrylisky.com/2014/03/03/installing-pcl-on-ubuntu/

A few updates to the instructions above were needed.

* libvtk needed to be updated to libvtk6-dev instead of (libvtk5-dev). The linker was having trouble locating libvtk5-dev while building, but this might not be a problem for everyone.

* BUILD_visualization needed to be manually turned on, this link shows you how to do that,
http://www.pointclouds.org/documentation/tutorials/building_pcl.php

