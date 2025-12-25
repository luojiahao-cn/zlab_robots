#include <zlab_environment/environment_loader.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <shape_msgs/SolidPrimitive.h>
#include <std_msgs/ColorRGBA.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/ObjectColor.h>
#include <cstdlib>
#include <sstream>

namespace zlab_environment
{

EnvironmentLoader::EnvironmentLoader(ros::NodeHandle& nh) : nh_(nh), planning_scene_interface_("")
{
  // 获取 arm_id 参数（用于动态替换 frame_id）
  if (!nh_.getParam("arm_id", arm_id_))
  {
    // 尝试从全局参数获取
    ros::NodeHandle nh_global;
    if (!nh_global.getParam("arm_id", arm_id_))
    {
      arm_id_ = "arm1";  // 默认值
      ROS_WARN("arm_id parameter not found, using default: %s", arm_id_.c_str());
    }
  }
  
  // 获取环境配置文件路径
  if (!nh_.getParam("environment_config", config_file_))
  {
    config_file_ = nh_.param<std::string>("environment_config", 
      "$(find zlab_environment)/config/default_environment.yaml");
  }
  
  ROS_INFO("Environment config file: %s", config_file_.c_str());
  ROS_INFO("Using arm_id: %s", arm_id_.c_str());
}

EnvironmentLoader::~EnvironmentLoader()
{
}

std::string EnvironmentLoader::expandPath(const std::string& path)
{
  std::string expanded = path;
  
  // 展开 $(find package_name)
  size_t pos = expanded.find("$(find ");
  while (pos != std::string::npos)
  {
    size_t end = expanded.find(")", pos);
    if (end != std::string::npos)
    {
      std::string package = expanded.substr(pos + 7, end - pos - 7);
      std::string package_path = ros::package::getPath(package);
      if (!package_path.empty())
      {
        expanded.replace(pos, end - pos + 1, package_path);
      }
      else
      {
        ROS_WARN("Package '%s' not found, using original path", package.c_str());
        break;
      }
    }
    else
    {
      break;
    }
    pos = expanded.find("$(find ", pos + 1);
  }
  
  // 展开 $(env VAR)
  pos = expanded.find("$(env ");
  while (pos != std::string::npos)
  {
    size_t end = expanded.find(")", pos);
    if (end != std::string::npos)
    {
      std::string var = expanded.substr(pos + 6, end - pos - 6);
      const char* env_value = std::getenv(var.c_str());
      if (env_value)
      {
        expanded.replace(pos, end - pos + 1, env_value);
      }
      else
      {
        ROS_WARN("Environment variable '%s' not found", var.c_str());
        break;
      }
    }
    else
    {
      break;
    }
    pos = expanded.find("$(env ", pos + 1);
  }
  
  return expanded;
}

bool EnvironmentLoader::loadEnvironmentConfig(const std::string& config_file)
{
  try
  {
    std::string expanded_path = expandPath(config_file);
    ROS_INFO("Loading environment from: %s", expanded_path.c_str());
    
    YAML::Node config = YAML::LoadFile(expanded_path);
    
    // 检查是否有单臂环境配置
    if (config["collision_objects"])
    {
      loadCollisionObjects(config["collision_objects"]);
    }
    
    // 检查是否有双臂环境配置
    if (config["dual_arm_collision_objects"])
    {
      loadCollisionObjects(config["dual_arm_collision_objects"]);
    }
    
    if (collision_objects_.empty())
    {
      ROS_WARN("No collision objects found in configuration file");
      return false;
    }
    
    return true;
  }
  catch (const YAML::Exception& e)
  {
    ROS_ERROR("YAML parsing error: %s", e.what());
    return false;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Error loading environment config: %s", e.what());
    return false;
  }
}

void EnvironmentLoader::loadCollisionObjects(const YAML::Node& objects_node)
{
  for (const auto& obj_node : objects_node)
  {
    moveit_msgs::CollisionObject collision_object;
    
    // 基本信息
    if (!obj_node["id"] || !obj_node["frame_id"] || !obj_node["type"])
    {
      ROS_WARN("Skipping collision object: missing required fields (id, frame_id, or type)");
      continue;
    }
    
    collision_object.id = obj_node["id"].as<std::string>();
    std::string frame_id = obj_node["frame_id"].as<std::string>();
    collision_object.header.frame_id = replaceFrameId(frame_id);
    collision_object.header.stamp = ros::Time::now();
    
    // 位置和姿态
    if (!obj_node["position"] || !obj_node["orientation"])
    {
      ROS_WARN("Skipping collision object '%s': missing position or orientation", collision_object.id.c_str());
      continue;
    }
    
    auto pos = obj_node["position"].as<std::vector<double>>();
    auto ori = obj_node["orientation"].as<std::vector<double>>();
    
    if (pos.size() != 3 || ori.size() != 4)
    {
      ROS_WARN("Skipping collision object '%s': invalid position or orientation size", collision_object.id.c_str());
      continue;
    }
    
    geometry_msgs::Pose pose;
    pose.position.x = pos[0];
    pose.position.y = pos[1];
    pose.position.z = pos[2];
    pose.orientation.x = ori[0];
    pose.orientation.y = ori[1];
    pose.orientation.z = ori[2];
    pose.orientation.w = ori[3];
    
    // 根据类型创建形状
    std::string type = obj_node["type"].as<std::string>();
    
    if (type == "box")
    {
      if (!obj_node["dimensions"])
      {
        ROS_WARN("Skipping collision object '%s': missing dimensions for box", collision_object.id.c_str());
        continue;
      }
      
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      auto dims = obj_node["dimensions"].as<std::vector<double>>();
      if (dims.size() != 3)
      {
        ROS_WARN("Skipping collision object '%s': box dimensions must have 3 values", collision_object.id.c_str());
        continue;
      }
      primitive.dimensions.resize(3);
      primitive.dimensions[0] = dims[0];  // x
      primitive.dimensions[1] = dims[1];  // y
      primitive.dimensions[2] = dims[2];  // z
      
      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(pose);
    }
    else if (type == "cylinder")
    {
      if (!obj_node["dimensions"])
      {
        ROS_WARN("Skipping collision object '%s': missing dimensions for cylinder", collision_object.id.c_str());
        continue;
      }
      
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.CYLINDER;
      auto dims = obj_node["dimensions"].as<std::vector<double>>();
      if (dims.size() != 2)
      {
        ROS_WARN("Skipping collision object '%s': cylinder dimensions must have 2 values (height, radius)", collision_object.id.c_str());
        continue;
      }
      primitive.dimensions.resize(2);
      primitive.dimensions[0] = dims[0];  // height
      primitive.dimensions[1] = dims[1];  // radius
      
      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(pose);
    }
    else if (type == "sphere")
    {
      if (!obj_node["dimensions"])
      {
        ROS_WARN("Skipping collision object '%s': missing dimensions for sphere", collision_object.id.c_str());
        continue;
      }
      
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.SPHERE;
      auto dims = obj_node["dimensions"].as<std::vector<double>>();
      if (dims.size() != 1)
      {
        ROS_WARN("Skipping collision object '%s': sphere dimensions must have 1 value (radius)", collision_object.id.c_str());
        continue;
      }
      primitive.dimensions.resize(1);
      primitive.dimensions[0] = dims[0];  // radius
      
      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(pose);
    }
    else
    {
      ROS_WARN("Unknown collision object type '%s' for object '%s', skipping", type.c_str(), collision_object.id.c_str());
      continue;
    }
    
    collision_object.operation = collision_object.ADD;
    collision_objects_.push_back(collision_object);

    // 解析可选颜色（r,g,b,a），若未提供则使用默认灰色
    std_msgs::ColorRGBA color;
    color.r = 0.6;
    color.g = 0.6;
    color.b = 0.6;
    color.a = 0.5;
    if (obj_node["color"])
    {
      auto c = obj_node["color"].as<std::vector<double>>();
      if (c.size() >= 3)
      {
        color.r = c[0];
        color.g = c[1];
        color.b = c[2];
        if (c.size() >= 4)
          color.a = c[3];
      }
    }
    object_colors_.push_back(std::make_pair(collision_object.id, color));
    
    ROS_INFO("Loaded collision object: %s (type: %s, frame: %s)", 
             collision_object.id.c_str(), type.c_str(), collision_object.header.frame_id.c_str());
  }
}

void EnvironmentLoader::addCollisionObjects()
{
  if (collision_objects_.empty())
  {
    ROS_WARN("No collision objects to add");
    return;
  }
  
  ROS_INFO("Adding %zu collision objects to planning scene", collision_objects_.size());
  
  // 等待 planning scene 准备好
  ros::Duration(1.0).sleep();
  
  // 添加到 planning scene
  planning_scene_interface_.addCollisionObjects(collision_objects_);

  // 设置颜色：通过 PlanningScene 消息应用
  moveit_msgs::PlanningScene ps_msg;
  ps_msg.is_diff = true;
  for (const auto& oc : object_colors_)
  {
    moveit_msgs::ObjectColor oc_msg;
    oc_msg.id = oc.first;
    oc_msg.color = oc.second;
    ps_msg.object_colors.push_back(oc_msg);
  }
  planning_scene_interface_.applyPlanningScene(ps_msg);
  
  // 等待物体被添加
  ros::Duration(0.5).sleep();
  
  ROS_INFO("Collision objects added successfully");
}

std::string EnvironmentLoader::replaceFrameId(const std::string& frame_id)
{
  std::string result = frame_id;
  
  // 替换 {arm_id} 占位符
  size_t pos = result.find("{arm_id}");
  if (pos != std::string::npos)
  {
    result.replace(pos, 8, arm_id_);
  }
  
  // 如果 frame_id 包含 arm1 或 arm2，但当前 arm_id 不同，则替换
  // 支持 base_link, pedestal_link 等
  if (result.find("arm1_") == 0 && arm_id_ != "arm1")
  {
    result.replace(0, 5, arm_id_);
  }
  else if (result.find("arm2_") == 0 && arm_id_ != "arm2")
  {
    result.replace(0, 5, arm_id_);
  }
  
  return result;
}

bool EnvironmentLoader::loadEnvironment()
{
  if (!loadEnvironmentConfig(config_file_))
  {
    ROS_ERROR("Failed to load environment configuration");
    return false;
  }
  
  addCollisionObjects();
  
  return true;
}

} // namespace zlab_environment

// Main function
int main(int argc, char** argv)
{
  ros::init(argc, argv, "environment_loader");
  ros::NodeHandle nh("~");
  
  zlab_environment::EnvironmentLoader loader(nh);
  
  if (loader.loadEnvironment())
  {
    ROS_INFO("Environment loaded successfully");
    // 保持节点运行，以便可以动态更新环境
    ros::spin();
  }
  else
  {
    ROS_ERROR("Failed to load environment");
    return 1;
  }
  
  return 0;
}
