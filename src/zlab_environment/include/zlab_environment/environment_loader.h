#ifndef ZLAB_ENVIRONMENT_ENVIRONMENT_LOADER_H
#define ZLAB_ENVIRONMENT_ENVIRONMENT_LOADER_H

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>

namespace zlab_environment
{

class EnvironmentLoader
{
public:
  EnvironmentLoader(ros::NodeHandle& nh);
  ~EnvironmentLoader();

  bool loadEnvironment();

private:
  std::string expandPath(const std::string& path);
  bool loadEnvironmentConfig(const std::string& config_file);
  void loadCollisionObjects(const YAML::Node& objects_node);
  void addCollisionObjects();
  std::string replaceFrameId(const std::string& frame_id);

  ros::NodeHandle nh_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::vector<moveit_msgs::CollisionObject> collision_objects_;
  std::vector<std::pair<std::string, std_msgs::ColorRGBA>> object_colors_;
  std::string config_file_;
  std::string arm_id_;  // 用于动态替换 frame_id
};

} // namespace zlab_environment

#endif // ZLAB_ENVIRONMENT_ENVIRONMENT_LOADER_H

