#include <fetch_grasp_suggestion/suggester.h>

using std::ios;
using std::string;
using std::stringstream;
using std::vector;

Suggester::Suggester() :
    pnh_("~"),
    pc_(new pcl::PointCloud<pcl::PointXYZRGB>),
    sample_grasps_client_("/rail_agile/sample_grasps"),
    sample_grasps_baseline_client_("/rail_agile/sample_classify_grasps"),
    rank_grasps_object_client_("/grasp_sampler/rank_grasps_object")
{
  string segmentation_topic;
  pnh_.param<string>("segmentation_topic", segmentation_topic, "rail_segmentation/segmented_objects");
  pnh_.param<string>("cloud_topic", cloud_topic_, "head_camera/depth_registered/points");
  pnh_.param<string>("file_name", filename_, "grasp_data");
  pnh_.param<double>("min_grasp_depth", min_grasp_depth_, -0.03);
  pnh_.param<double>("max_grasp_depth", max_grasp_depth_, 0.03);

  stringstream ss;
  ss << filename_ << ".csv";
  filename_ = ss.str();

  suggest_grasps_service_ = pnh_.advertiseService("suggest_grasps", &Suggester::suggestGraspsCallback, this);
}

bool Suggester::suggestGraspsCallback(rail_manipulation_msgs::SuggestGrasps::Request &req,
    rail_manipulation_msgs::SuggestGrasps::Response &res)
{
  boost::mutex::scoped_lock grasp_lock(stored_grasp_mutex_);

  stored_grasp_list_.grasps.clear();
  stored_grasp_list_.object_index = 0;
  object_features_.clear();
  selected_grasps_.clear();

  stored_object_cloud_ = req.cloud;

  // get the current point cloud (for collision checking)
  ros::Time request_time = ros::Time::now();
  ros::Time point_cloud_time = request_time - ros::Duration(0.1);
  while (point_cloud_time < request_time)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc_msg =
        ros::topic::waitForMessage< pcl::PointCloud<pcl::PointXYZRGB> >(cloud_topic_, n_, ros::Duration(30.0));
    if (pc_msg == NULL)
    {
      ROS_INFO("No point cloud received for segmentation.");
      return false;
    }
    else
    {
      *pc_ = *pc_msg;
    }
    point_cloud_time = pcl_conversions::fromPCL(pc_->header.stamp);
  }

  //save frames for lots of upcoming point cloud transforming
  string environment_source_frame = pc_->header.frame_id;
  string object_source_frame = stored_object_cloud_.header.frame_id;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  geometry_msgs::PoseArray sampled_grasps;
  SampleGraspCandidates(stored_object_cloud_, object_source_frame, environment_source_frame,
                        sampled_grasps, cropped_cloud);

  ROS_INFO("Sampling complete.");

  if (!sampled_grasps.poses.empty())
  {
    ROS_INFO("Ranking grasp candidates...");
    rankCandidates(cropped_cloud, stored_object_cloud_, sampled_grasps, object_source_frame, stored_grasp_list_);

    // remove any grasps with fingers in collision with object
    for (int i = static_cast<int>(stored_grasp_list_.grasps.size()) - 1; i >= 0; i --)
    {
      if (isInCollision(stored_grasp_list_.grasps[i].pose, stored_object_cloud_, false))
      {
        stored_grasp_list_.grasps.erase(stored_grasp_list_.grasps.begin() + i);
      }
    }

    ROS_INFO("%lu grasps remain after collision checking", stored_grasp_list_.grasps.size());

    // TODO: this is removed and moved to after pairwise ranking for competition optimization only
//    // iteratively calculate grasp depth
//    for (int i = 0; i < stored_grasp_list_.grasps.size(); i ++)
//    {
//      double depth_lower_bound = min_grasp_depth_;
//      double depth_upper_bound = max_grasp_depth_;
//      double current_depth = max_grasp_depth_;
//      geometry_msgs::PoseStamped test_pose = stored_grasp_list_.grasps[i].pose;
//
//      // check max grasp depth first
//      test_pose.pose = adjustGraspDepth(stored_grasp_list_.grasps[i].pose.pose, current_depth);
//      if (isInCollision(test_pose, pc_, true))
//      {
//        geometry_msgs::Pose adjustedPose;
//        // binary search to set grasp pose
//        for (int j = 0; j < 5; j++)
//        {
//          current_depth = (depth_lower_bound + depth_upper_bound) / 2.0;
//          adjustedPose = adjustGraspDepth(stored_grasp_list_.grasps[i].pose.pose, current_depth);
//          test_pose.pose.position.x = adjustedPose.position.x;
//          test_pose.pose.position.y = adjustedPose.position.y;
//          test_pose.pose.position.z = adjustedPose.position.z;
//          if (isInCollision(test_pose, pc_, true))
//          {
//            depth_upper_bound = current_depth;
//          }
//          else
//          {
//            depth_lower_bound = current_depth;
//          }
//        }
//      }
//      stored_grasp_list_.grasps[i].pose.pose.position.x = test_pose.pose.position.x;
//      stored_grasp_list_.grasps[i].pose.pose.position.y = test_pose.pose.position.y;
//      stored_grasp_list_.grasps[i].pose.pose.position.z = test_pose.pose.position.z;
//    }

    if (!stored_grasp_list_.grasps.empty())
    {
      res.grasp_list.header.frame_id = stored_grasp_list_.grasps[0].pose.header.frame_id;
      for (size_t i = 0; i < stored_grasp_list_.grasps.size(); i ++)
      {
        res.grasp_list.poses.push_back(stored_grasp_list_.grasps[i].pose.pose);
      }
    }
  }
  return true;
}

void Suggester::SampleGraspCandidates(sensor_msgs::PointCloud2 object, string object_source_frame,
    string environment_source_frame, geometry_msgs::PoseArray &grasps_out,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out)
{
  return SampleGraspCandidates(object, object_source_frame, environment_source_frame, grasps_out, cloud_out, false);
}

void Suggester::SampleGraspCandidates(sensor_msgs::PointCloud2 object, string object_source_frame,
    string environment_source_frame, geometry_msgs::PoseArray &grasps_out,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, bool agile_only)
{
  //get a pcl version of the object point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2::Ptr temp_cloud(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(object, *temp_cloud);
  pcl::fromPCLPointCloud2(*temp_cloud, *object_cloud);

  //transform object cloud to camera frame to get new crop box dimensions
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (object_cloud->header.frame_id != environment_source_frame)
  {
    pcl_ros::transformPointCloud(environment_source_frame, ros::Time(0), *object_cloud, object_source_frame,
                                 *transformed_cloud, tf_listener_);
    transformed_cloud->header.frame_id = environment_source_frame;
  }
  else
  {
    *transformed_cloud = *object_cloud;
  }

  //calculate workspace bounds in new coordinate frame
  pcl::PointXYZRGB min_workspace_point, max_workspace_point;
  pcl::getMinMax3D(*transformed_cloud, min_workspace_point, max_workspace_point);

  //crop cloud based on specified object
  double cloud_padding = 0.03;
  pcl::CropBox<pcl::PointXYZRGB> crop_box;
  Eigen::Vector4f min_point, max_point;
  min_point[0] = static_cast<float>(min_workspace_point.x - cloud_padding);
  min_point[1] = static_cast<float>(min_workspace_point.y - cloud_padding);
  min_point[2] = static_cast<float>(min_workspace_point.z - cloud_padding);
  max_point[0] = static_cast<float>(max_workspace_point.x + cloud_padding);
  max_point[1] = static_cast<float>(max_workspace_point.y + cloud_padding);
  max_point[2] = static_cast<float>(max_workspace_point.z + cloud_padding);
  crop_box.setMin(min_point);
  crop_box.setMax(max_point);
  crop_box.setInputCloud(pc_);
  crop_box.filter(*cloud_out);

  rail_grasp_calculation_msgs::SampleGraspsGoal sample_goal;
  pcl::toPCLPointCloud2(*cloud_out, *temp_cloud);
  pcl_conversions::fromPCL(*temp_cloud, sample_goal.cloud);

  sample_goal.workspace.mode = rail_grasp_calculation_msgs::Workspace::WORKSPACE_VOLUME;
  sample_goal.workspace.x_min = min_point[0];
  sample_goal.workspace.y_min = min_point[1];
  sample_goal.workspace.z_min = min_point[2];
  sample_goal.workspace.x_max = max_point[0];
  sample_goal.workspace.y_max = max_point[1];
  sample_goal.workspace.z_max = max_point[2];

  if (agile_only)
  {
    sample_grasps_baseline_client_.sendGoal(sample_goal);
    sample_grasps_baseline_client_.waitForResult(ros::Duration(10.0));
    grasps_out = sample_grasps_baseline_client_.getResult()->graspList;
    return;
  }

  sample_grasps_client_.sendGoal(sample_goal);
  sample_grasps_client_.waitForResult(ros::Duration(10.0));
  grasps_out = sample_grasps_client_.getResult()->graspList;
}

void Suggester::rankCandidates(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, sensor_msgs::PointCloud2 object_cloud,
    geometry_msgs::PoseArray &grasps, string object_source_frame,
    fetch_grasp_suggestion::RankedGraspList &ranked_grasps)
{
  //transform environment cloud back to object frame
  sensor_msgs::PointCloud2 environment_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2::Ptr temp_cloud(new pcl::PCLPointCloud2);
  if (cloud_in->header.frame_id != object_source_frame)
  {
    pcl_ros::transformPointCloud(object_source_frame, ros::Time(0), *cloud_in, object_source_frame, *transformed_cloud,
                                 tf_listener_);
    transformed_cloud->header.frame_id = object_source_frame;
    pcl::toPCLPointCloud2(*transformed_cloud, *temp_cloud);
  }
  else
  {
    pcl::toPCLPointCloud2(*cloud_in, *temp_cloud);
  }
  pcl_conversions::fromPCL(*temp_cloud, environment_cloud);

  rail_grasp_calculation_msgs::RankGraspsGoal rank_goal;
  rank_goal.sceneCloud = environment_cloud;
  rank_goal.segmentedCloud = object_cloud;
  rank_goal.graspList = grasps;
  rank_grasps_object_client_.sendGoal(rank_goal);

  rank_grasps_object_client_.waitForResult(ros::Duration(10.0));
  rail_grasp_calculation_msgs::RankGraspsResultConstPtr rank_result = rank_grasps_object_client_.getResult();

  ranked_grasps.grasps.clear();
  if (!rank_result->graspList.poses.empty())
  {
    ranked_grasps.grasps.resize(rank_result->graspList.poses.size());
    for (size_t i = 0; i < rank_result->graspList.poses.size(); i++)
    {
      ranked_grasps.grasps[i].pose.header.frame_id = rank_result->graspList.header.frame_id;
      ranked_grasps.grasps[i].pose.pose = rank_result->graspList.poses[i];
      ranked_grasps.grasps[i].heuristics = rank_result->heuristicList[i].heuristics;
    }

    //rotate each pose by 90 degree roll to align AGILE's results with gripper's coordinate frame
    tf::Quaternion rotation_adjustment = tf::createQuaternionFromRPY(M_PI / 2.0, 0, 0);
    for (size_t i = 0; i < ranked_grasps.grasps.size(); i++)
    {
      tf::Quaternion q;
      tf::quaternionMsgToTF(ranked_grasps.grasps[i].pose.pose.orientation, q);
      tf::Quaternion adjusted_q = q * rotation_adjustment;
      tf::quaternionTFToMsg(adjusted_q, ranked_grasps.grasps[i].pose.pose.orientation);
    }
  }
  else
  {
    ROS_INFO("Didn't receive any ranked grasps...");
  }
}

geometry_msgs::Pose Suggester::adjustGraspDepth(geometry_msgs::Pose grasp_pose, double distance)
{
  geometry_msgs::Pose result;
  result.orientation.x = grasp_pose.orientation.x;
  result.orientation.y = grasp_pose.orientation.y;
  result.orientation.z = grasp_pose.orientation.z;
  result.orientation.w = grasp_pose.orientation.w;

  geometry_msgs::Transform transform;
  Eigen::Affine3d transform_matrix;
  transform.rotation = grasp_pose.orientation;
  transform.translation.x = grasp_pose.position.x;
  transform.translation.y = grasp_pose.position.y;
  transform.translation.z = grasp_pose.position.z;
  tf::transformMsgToEigen(transform, transform_matrix);

  Eigen::Vector3d transform_point, transformed_point;
  transform_point[0] = distance;
  transform_point[1] = 0;
  transform_point[2] = 0;
  transformed_point = transform_matrix*transform_point;
  tf::pointEigenToMsg(transformed_point, result.position);

  return result;
}

bool Suggester::isInCollision(geometry_msgs::PoseStamped grasp, sensor_msgs::PointCloud2 cloud, bool check_palm)
{
  // convert to pcl point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2::Ptr temp_cloud(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(cloud, *temp_cloud);
  pcl::fromPCLPointCloud2(*temp_cloud, *cloud_pcl);

  return isInCollision(grasp, cloud_pcl, check_palm);
}

bool Suggester::isInCollision(geometry_msgs::PoseStamped grasp, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    bool check_palm)
{
  //TODO(enhancement): finger tip size, shape, and position are all hardcoded for Fetch
  pcl::CropBox<pcl::PointXYZRGB> crop_box;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr collision_points(new pcl::PointCloud<pcl::PointXYZRGB>);
  Eigen::Vector4f min_point, max_point;
  crop_box.setInputCloud(cloud);

  geometry_msgs::PoseStamped check_grasp = grasp;
  if (grasp.header.frame_id != cloud->header.frame_id)
  {
    // transform grasp pose to point cloud frame
    tf_listener_.transformPose(cloud->header.frame_id, ros::Time(0), grasp, cloud->header.frame_id, check_grasp);
    check_grasp.header.frame_id = cloud->header.frame_id;
  }

  geometry_msgs::Transform transform;
  Eigen::Affine3d transform_matrix;
  transform.rotation = check_grasp.pose.orientation;
  transform.translation.x = check_grasp.pose.position.x;
  transform.translation.y = check_grasp.pose.position.y;
  transform.translation.z = check_grasp.pose.position.z;
  tf::transformMsgToEigen(transform, transform_matrix);

  Eigen::Vector3d transform_point;
  Eigen::Vector3d transformed_point;
  tf::Quaternion rotation_tf;
  double r, p, y;
  Eigen::Vector3f rotation;
  geometry_msgs::PoseStamped test_pose;
  visualization_msgs::Marker test_marker;

  // left finger
  min_point[0] = -0.029f;
//  max_point[0] = 0.029f;
  max_point[0] = 0.029f + 0.000f;  // making fingers slightly longer to prevent fingertips hitting surfaces
  min_point[1] = 0;
  max_point[1] = 0.014f;
  min_point[2] = -0.013f;
  max_point[2] = 0.013f;
  crop_box.setMin(min_point);
  crop_box.setMax(max_point);

  transform_point[0] = 0;
  transform_point[1] = -0.065f;
  transform_point[2] = 0;
  transformed_point = transform_matrix*transform_point;
  crop_box.setTranslation(transformed_point.cast<float>());

  tf::quaternionMsgToTF(check_grasp.pose.orientation, rotation_tf);
  tf::Matrix3x3(rotation_tf).getRPY(r, p, y);
  rotation[0] = static_cast<float>(r);
  rotation[1] = static_cast<float>(p);
  rotation[2] = static_cast<float>(y);
  crop_box.setRotation(rotation);

  crop_box.filter(*collision_points);
  if (!collision_points->points.empty())
    return true;

  // right finger
  min_point[1] = -0.014f;
  max_point[1] = 0;
  crop_box.setMin(min_point);
  crop_box.setMax(max_point);

  transform_point[1] = 0.065f;
  transformed_point = transform_matrix*transform_point;
  crop_box.setTranslation(transformed_point.cast<float>());

  crop_box.filter(*collision_points);
  if (!collision_points->points.empty())
    return true;

  if (check_palm)
  {
    min_point[0] = 0;
    max_point[0] = 0.137f;
    min_point[1] = -0.059f;
    max_point[1] = 0.059f;
    min_point[2] = -0.035f;
    max_point[2] = 0.035f;
    crop_box.setMin(min_point);
    crop_box.setMax(max_point);

    transform_point[0] = -0.166;
    transform_point[1] = 0;
    transform_point[2] = 0;
    transformed_point = transform_matrix*transform_point;
    crop_box.setTranslation(transformed_point.cast<float>());

    crop_box.filter(*collision_points);
    if (!collision_points->points.empty())
      return true;
  }

  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "suggester");

  Suggester s;

  ros::spin();

  return EXIT_SUCCESS;
}
