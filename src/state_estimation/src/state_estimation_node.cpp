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
* file: state_estimation_node.cpp
* auth: Alyssa Wing
* desc: state estimation node for ros2
*/

#include <state_estimation_node.hpp>

namespace utfr_dv{
namespace state_estimation{

StateEstimationNode::StateEstimationNode():Node("state_estimation_node")
  {
   this->initParams();
   this->initPublishers();
   this->initSubscribers();

   this->initHeartbeat();

   this->initTransforms();

   this->initTimers();

  }

void StateEstimationNode::initParams(){
  //initializing parameters with default values
  this->declare_parameter("update_rate", 1000.0);
  this->declare_parameter("publish_string","Default Message");
  //this->declare_parameter("echo_string", "");//node loop timer?

  //load parameters
  publish_string_ = 
      this->get_parameter("publish_string").get_parameter_value().get<
          std::string>();
  update_rate_ =
          this->get_parameter("update_rate").get_parameter_value().get<double>();
  // echo_string_ = 
  //     this->get_parameter("echo_string").get_parameter_value().get<
  //         std::string>();
}

void StateEstimationNode::initPublishers(){
  cone_map_publisher_ = 
    this->create_publisher<utfr_msgs::msg::ConeMap>(topics::kConeMap, 10);

  ego_state_publisher_ = 
    this->create_publisher<utfr_msgs::msg::EgoState>(topics::kEgoState, 10);

  rclcpp::QoS custom_qos_profile(100);
  custom_qos_profile.keep_last(1);
  custom_qos_profile.transient_local();

  world_datum_publisher_ = 
    this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
          topics::kDatumLLA, custom_qos_profile);

  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
          topics::kStateEstimationHeartbeat,1); 
}

void StateEstimationNode::initSubscribers(){
  // filtered_gps_subscriber_ = 
  //     this->create_subscription<std_msgs::msg::String>(
  //         topics::kGPS, 1, //TODO: change to real topic name
  //         std::bind(&StateEstimationNode::filteredGPSCB, this, 
  //             std::placeholders::_1));
  cone_detections_subscriber_ =     
      this->create_subscription<utfr_msgs::msg::ConeDetections>(
          topics::kConeDetections, 1, 
          std::bind(&StateEstimationNode::coneDetectionsCB, this, 
              std::placeholders::_1));

  gps_subscriber_ =     
      this->create_subscription<geometry_msgs::msg::Vector3>(
          topics::kGPSData, 1, 
          std::bind(&StateEstimationNode::gpsCB, this, 
              std::placeholders::_1));
}

void StateEstimationNode::initTimers(){
  main_timer_ = this->create_wall_timer(
      std::chrono::duration<double,std::milli>(update_rate_), 
          std::bind(&StateEstimationNode::timerCB, this));
}

void StateEstimationNode::initTransforms(){
  tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  transform_.header.frame_id = frames::kWorld;
  transform_.child_frame_id = frames::kBase;

  // Initialize transformation with zero distplcement and rotation
  transform_.transform.translation.x = 0.0;
  transform_.transform.translation.y = 0.0;
  transform_.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  transform_.transform.rotation.x = q.x();
  transform_.transform.rotation.y = q.y();
  transform_.transform.rotation.z = q.z();
  transform_.transform.rotation.w = q.w();

  // Send the transformation
  tf_broadcaster_->sendTransform(transform_);
}

void StateEstimationNode::coneDetectionsCB(
      const utfr_msgs::msg::ConeDetections& msg){
  utfr_msgs::msg::ConeDetections current_detections_ = msg;
}

void StateEstimationNode::initHeartbeat(){
      heartbeat_.module.data = "state_estimation";
      heartbeat_.update_rate = update_rate_;
    }

void StateEstimationNode::publishHeartbeat(const int status){
  heartbeat_.status = status;
  heartbeat_.header.stamp = this->get_clock()->now();
  heartbeat_publisher_->publish(heartbeat_);
}

void StateEstimationNode::publishDatum(const geometry_msgs::msg::Vector3& datum)
  {
  const std::string function_name{"publishDatum: "};
  geometry_msgs::msg::Vector3Stamped datum_msg;
  world_datum_ = std::make_shared<geometry_msgs::msg::Vector3>();

  datum_msg.header.stamp = this->get_clock()->now();
  world_datum_->x = datum.x;
  world_datum_->y = datum.y;
  world_datum_->z = datum.z;
  datum_msg.vector = *world_datum_;

  RCLCPP_INFO(this->get_logger(), 
      "%s Published Datum!", function_name.c_str());

  world_datum_publisher_->publish(datum_msg);
}

void StateEstimationNode::gpsCB(const geometry_msgs::msg::Vector3& location) {

  if (world_datum_ == nullptr && curr_gps_msg_ == nullptr) {
    curr_gps_msg_ = std::make_shared<geometry_msgs::msg::Vector3>();
    this->publishDatum(location);
  }

  curr_gps_msg_->x = location.x;
  curr_gps_msg_->y = location.y;
  curr_gps_msg_->z = location.z;
}
void StateEstimationNode::timerCB(){
  auto curr_message = std_msgs::msg::String();
  curr_message.data = publish_string_ + std::to_string(counter_);
  //ego_state_publisher_->publish(curr_message);

  RCLCPP_INFO(this->get_logger(), 
      "Published Message:  %s!", curr_message.data.c_str());
  
  auto counter_factorial = util::factorial(counter_);

  RCLCPP_INFO(this->get_logger(), 
      "Counter Factorial::  %s!", std::to_string(counter_factorial).c_str());

  counter_++;

  //TODO - change status based on operation
  this->publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);

  // Set Transform
  if (current_ego_state_ != nullptr) {
    this->updateTransforms(*current_ego_state_);
  }

}

void StateEstimationNode::updateTransforms(
    const utfr_msgs::msg::EgoState& curr_state){

  //Update Transform with latest ego state relative to world origin
  transform_.transform.translation.x = curr_state.pos.x;
  transform_.transform.translation.y = curr_state.pos.y;
  transform_.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, curr_state.yaw);
  transform_.transform.rotation.x = q.x();
  transform_.transform.rotation.y = q.y();
  transform_.transform.rotation.z = q.z();
  transform_.transform.rotation.w = q.w();

  // Send the transformation
  tf_broadcaster_->sendTransform(transform_);

}
} //namespace state_estimation
} //namespace utfr_dv
