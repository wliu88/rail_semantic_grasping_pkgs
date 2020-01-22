# rail_semantic_grasping_pkgs
Temporary repo for migrating robot code to ros kinetic. 

This repo contains simplified version of `fetch_grasp_suggestion`, which only supports 
  * `~/suggest_grasps`([rail_manipulation_msgs/SuggestGrasps](https://github.com/GT-RAIL/rail_manipulation_msgs/blob/master/srv/SuggestGrasps.srv))
  Given an object point cloud, sample grasps and get an initial ranking for them by
  calculating grasp heuristics.
  
The `rail_semantic_grasping` repo remains the same. Check the readme inside for more detail.

## Installation
Install this pacakage along with the following packages:
* [`rail_agile`](https://github.com/GT-RAIL/rail_agile)
* [`rail_grasp_calculation`](https://github.com/GT-RAIL/rail_grasp_calculation)
* [`rail_manipulation_msgs`](https://github.com/GT-RAIL/rail_manipulation_msgs)
* [`rail_segmentation`](https://github.com/GT-RAIL/rail_segmentation)

The melodic versions of these packages should also compile in kinetic. 

## Note:
If `isnan` error is encountered when compiling `rail_agile`, add `#include <cmath>` to header and replace `isnan` with `std::isnan` in code.

## Run
See readme inside `rail_semantic_grasping`
  
