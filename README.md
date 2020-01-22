# rail_semantic_grasping_pkgs
Temporary repo for migrating robot code to ros kinetic. 

This repo contains simplified version of `fetch_grasp_suggestion`, which only supports 
  * `~/suggest_grasps`([rail_manipulation_msgs/SuggestGrasps](https://github.com/GT-RAIL/rail_manipulation_msgs/blob/master/srv/SuggestGrasps.srv))
  Given an object point cloud, sample grasps and get an initial ranking for them by
  calculating grasp heuristics.
  
The `rail_semantic_grasping` repo remains the same. Check the readme inside for more detail.
  
