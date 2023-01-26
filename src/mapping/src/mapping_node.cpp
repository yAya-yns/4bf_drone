/*
******************************************************
|                                                    |
|           _     __                    _            |
|    _   _ | |_  / _| _ __            _| |__   __    |
|   | | | || __|| |_ | '__|         / _` |\ \ / /    |
|   | |_| || |_ |  _|| |           | (_| | \ V /     |
|    \__,_| \__||_|  |_|    _____   \__,_|  \_/      |
|                          |_____|                   |
|                                                    |
|                                                    |
******************************************************
*
* file: mapping_node.cpp
* auth: Kelvin Cui
* desc: mapping node class
*/

#include <mapping_node.hpp>

namespace utfr_dv {
namespace mapping{

MappingNode::MappingNode() : Node("mapping_node"){
  this->initParams();
  this->initSubscribers();
  this->initPublishers();
  this->initTransforms();
  this->initHeartbeat();
  this->initFusion();
  this->initTimers();
  }

void MappingNode::initParams(){
  // Initialize Params with default values
  this->declare_parameter("update_rate", 33.33);

  update_rate_ = this->get_parameter(
    "update_rate").as_double();
}

void MappingNode::initSubscribers(){
  cone_detections_subscriber_ = 
      this->create_subscription<utfr_msgs::msg::ConeDetections>(
          topics::kConeDetections, 10, 
          std::bind(&MappingNode::coneDetectionsCB, this, _1));
  
  lidar_detections_subscriber_ = 
      this->create_subscription<sensor_msgs::msg::LaserScan>(
          topics::kLidarDetections, 10, 
          std::bind(&MappingNode::lidarDetectionsCB, this, _1));
}

void MappingNode::initPublishers(){
  cone_map_publisher_ = 
      this->create_publisher<utfr_msgs::msg::ConeMap>(topics::kConeMap, 10);

  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kMappingHeartbeat,1);
}

void MappingNode::initTransforms(){
  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void MappingNode::initFusion(){
  fusion_ = std::make_unique<Fusion>();
}

void MappingNode::initTimers(){
  main_timer_ = this->create_wall_timer(
      std::chrono::duration<double,std::milli>(update_rate_),
      std::bind(&MappingNode::timerCB, this));
}

void MappingNode::coneDetectionsCB(const utfr_msgs::msg::ConeDetections& msg){
  utfr_msgs::msg::ConeDetections cone_detections_world;

  const std::string function_name{"coneDetectionsCB:"};

  std::string toFrame = frames::kWorld;

  // Find the camera to world frame transform when the camera image was captured
  try {
    // Attempt to find transform within 50ms
     geometry_msgs::msg::TransformStamped transform_camera_to_world = 
        tf_buffer_->lookupTransform(
            toFrame, msg.header.stamp,
            msg.header.frame_id, msg.header.stamp,
            frames::kWorld, 
            std::chrono::duration<double,std::milli>(50));

    for (const auto& cone : msg.left_cones){
      utfr_msgs::msg::Cone cone_world = cone;
      
      //Transform vector3 of cone position from camera frame to world frame
      tf2::doTransform(cone.pos, 
                       cone_world.pos, 
                       transform_camera_to_world);

      cone_detections_world.left_cones.push_back(cone_world);
    }
    for (const auto& cone : msg.right_cones){
      utfr_msgs::msg::Cone cone_world = cone;
      
      tf2::doTransform(cone.pos, 
                       cone_world.pos, 
                       transform_camera_to_world);

      cone_detections_world.right_cones.push_back(cone_world); 
    }
    for (const auto& cone : msg.other_cones){
      utfr_msgs::msg::Cone cone_world = cone;
      
      tf2::doTransform(cone.pos, 
                       cone_world.pos, 
                       transform_camera_to_world);

      cone_detections_world.other_cones.push_back(cone_world); 
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "%s Could not transform %s to %s: %s",
      function_name.c_str(),
      toFrame.c_str(), msg.header.frame_id.c_str(), ex.what());
    return;
  }

  // Change header frame now that transform is complete
  cone_detections_world.header.frame_id = toFrame;
  cone_detections_world.header.stamp = msg.header.stamp;
  current_camera_ = 
      std::make_shared<utfr_msgs::msg::ConeDetections>(cone_detections_world);
  new_camera_detections_ = true;
}

void MappingNode::lidarDetectionsCB(const sensor_msgs::msg::LaserScan& msg){
  sensor_msgs::msg::LaserScan lidar_detections_world = msg;
  
  // TODO - Use State Estimation transform, 
  // transform from lidar frame to world frame
  // transform world frame lidar to coneDetections and assign to current_lidar_

  new_lidar_detections_ = true;
}

void MappingNode::initHeartbeat(){
  heartbeat_.module.data = "mapping";
  heartbeat_.update_rate = update_rate_;
}

void MappingNode::publishHeartbeat(const int status){
  heartbeat_.status = status;
  heartbeat_.header.stamp = this->get_clock()->now();
  heartbeat_publisher_->publish(heartbeat_);
}

void MappingNode::timerCB(){

  const std::string function_name {"MappingNode::timerCB:"};

  // If no data has arrived yet:
  if (current_camera_ == nullptr){
     RCLCPP_WARN(this->get_logger(), 
        "%s camera detections not ready!", function_name.c_str());
    return;
  }

  if (current_lidar_ == nullptr){
    RCLCPP_WARN(this->get_logger(), 
        "%s lidar detections not ready!", function_name.c_str());
    return;
  }

  // If no new data to proces - publish last map if exists
  if (!new_lidar_detections_ && !new_camera_detections_){
    this->publishHeartbeat(utfr_msgs::msg::Heartbeat::UNINITIALIZED);
    RCLCPP_WARN(this->get_logger(), 
        "%s no new data", function_name.c_str());
    if (current_map_ != nullptr){
      this->publishMap();
    }
    return;
  }

  this->updateMap();
  this->publishMap();
  this->publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);
}

void MappingNode::updateMap(){
  if (current_map_ == nullptr){
    utfr_msgs::msg::ConeMap template_map;
    template_map.header.frame_id = "world";
    current_map_ = std::make_shared<utfr_msgs::msg::ConeMap>(template_map);
  }
  
  // TODO - create latest map here:
  //current_map_ = fusion_->extractMatchedCones()
}

void MappingNode::publishMap(){
  
  current_map_->header.stamp = this->get_clock()->now();
  cone_map_publisher_->publish(*current_map_);

  // Reset new data flags
  new_lidar_detections_ = false;
  new_camera_detections_ = false;
}

} //namespace mapping
} //namespace utfr_dv