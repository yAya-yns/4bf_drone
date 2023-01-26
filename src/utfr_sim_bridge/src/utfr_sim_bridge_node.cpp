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
* file: utfr_sim_bridge_node.cpp
* auth: Youssef Elhadad
* desc: Utfr Sim Bridge publisher and subscriber node class for ros2 template
*/
#include <utfr_sim_bridge_node.hpp>

namespace utfr_dv {
namespace utfr_sim_bridge {
  
UTFRSimBridge::UTFRSimBridge():Node("utfr_sim_bridge_node"){
    this->initPublishers();
    this->initSubscribers();
    this->initClients();
}


void UTFRSimBridge::initPublishers(){

  control_cmd_publish_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      topics::kEUFSControlCmd,1);

  state_estimation_publisher_ = this->create_publisher<utfr_msgs::msg::EgoState>(
      topics::kEgoState,1);

  cone_detections_publisher_ = this->create_publisher<utfr_msgs::msg::ConeDetections>(
      topics::kConeDetections,1);

  target_state_publisher_ = this->create_publisher<utfr_msgs::msg::TargetState>(
      topics::kTargetState,1);
}

void UTFRSimBridge::initClients(){
  mission_client_ = this->create_client<eufs_msgs::srv::SetCanState>(
      topics::kEUFSMissionServer);

  //Set Mission
  auto request = std::make_shared<eufs_msgs::srv::SetCanState::Request>();
  request->as_state = eufs_msgs::msg::CanState::AS_DRIVING;
  request->ami_state = eufs_msgs::msg::CanState::AMI_ACCELERATION;
  int waiting = -1;
  while (!mission_client_->wait_for_service(1s)&&(waiting++<10)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = mission_client_->async_send_request(request);

}

void UTFRSimBridge::initSubscribers(){
  control_cmd_subscriber_ = 
    this->create_subscription<utfr_msgs::msg::ControlCmd>(
        topics::kControlCmd, 1, 
        std::bind(&UTFRSimBridge::controlCmdCB, this, _1));

  ego_state_subscriber_ = 
    this->create_subscription<eufs_msgs::msg::CarState>(
          topics::kEUFSEgoState, 1, 
          std::bind(&UTFRSimBridge::egoStateCB, this, _1));
  
  cone_detections_subscriber_ = 
    this->create_subscription<eufs_msgs::msg::ConeArrayWithCovariance>(
          topics::kEUFSConeDetection, 1, 
          std::bind(&UTFRSimBridge::coneDetectionsCB, this, _1));

  teensy_subscriber_ = 
    this->create_subscription<utfr_msgs::msg::Teensy>(
          topics::kTeensy, 1, 
          std::bind(&UTFRSimBridge::teensyCB, this, _1));
}

void UTFRSimBridge::controlCmdCB(
    const utfr_msgs::msg::ControlCmd& msg){

  RCLCPP_INFO(this->get_logger(),"UTFRSimBridge::controlCmdCB: Published Steering Command");

  ackermann_msgs::msg::AckermannDriveStamped new_cmd;
  new_cmd.drive.acceleration = msg.acceleration;
  new_cmd.drive.steering_angle = msg.str_cmd;

  control_cmd_publish_->publish(new_cmd);
}

void UTFRSimBridge::egoStateCB(
    const eufs_msgs::msg::CarState& msg){
  
  utfr_msgs::msg::EgoState new_ego_state;
  new_ego_state.pos.x = msg.pose.pose.position.x;
  new_ego_state.pos.y = msg.pose.pose.position.y;
  new_ego_state.pos.z = msg.pose.pose.position.z;
  new_ego_state.yaw = msg.pose.pose.position.z;
  new_ego_state.yaw_rate = msg.twist.twist.angular.z;

  new_ego_state.vel = msg.twist.twist.linear;
  new_ego_state.accel = msg.linear_acceleration;

  state_estimation_publisher_->publish(new_ego_state);
  RCLCPP_INFO(this->get_logger(),"egoStateCB: Published EgoState.pos.x = %f", new_ego_state.pos.x);
}

void UTFRSimBridge::coneDetectionsCB(
    const eufs_msgs::msg::ConeArrayWithCovariance& msg){

  RCLCPP_INFO(this->get_logger(),"coneDetectionsCB");
  utfr_msgs::msg::ConeDetections new_cd_message;
  if(std::begin(msg.blue_cones) == std::end(msg.blue_cones))
    return;
  long number_of_blue = std::end(msg.blue_cones) - std::begin(msg.blue_cones);
  for(long i = 0; i < number_of_blue; ++i) {
    new_cd_message.left_cones[i].pos.x = msg.blue_cones[i].point.x;
    new_cd_message.left_cones[i].pos.y = msg.blue_cones[i].point.y;
    new_cd_message.left_cones[i].type = utfr_msgs::msg::Cone::BLUE;
  }
  
  long number_of_yellow = std::end(msg.yellow_cones) - std::begin(msg.yellow_cones);
  for(long i = 0; i < number_of_yellow; ++i) {
    new_cd_message.right_cones[i].pos.x = msg.yellow_cones[i].point.x;
    new_cd_message.right_cones[i].pos.y = msg.yellow_cones[i].point.y;
    new_cd_message.right_cones[i].type = utfr_msgs::msg::Cone::BLUE;
  }

  long number_of_orange = std::end(msg.orange_cones) - std::begin(msg.orange_cones);
  for(long i = 0; i < number_of_orange; ++i) {
    new_cd_message.other_cones[i].pos.x = msg.orange_cones[i].point.x;
    new_cd_message.other_cones[i].pos.y = msg.orange_cones[i].point.y;
    new_cd_message.other_cones[i].type = utfr_msgs::msg::Cone::SMALL_ORANGE;
  }

  long number_of_all_orange = std::end(msg.big_orange_cones) - std::begin(msg.big_orange_cones) + number_of_orange;
  for(long i = number_of_orange; i < number_of_all_orange; ++i) {
    new_cd_message.other_cones[i].pos.x = msg.big_orange_cones[i].point.x;
    new_cd_message.other_cones[i].pos.y = msg.big_orange_cones[i].point.y;
    new_cd_message.other_cones[i].type = utfr_msgs::msg::Cone::LARGE_ORANGE;
  } 

  long number_of_others = std::end(msg.unknown_color_cones) - std::begin(msg.unknown_color_cones)  + sizeof(msg.unknown_color_cones);
  for(long i = number_of_all_orange; i < number_of_others; ++i) {
    new_cd_message.other_cones[i].pos.x = msg.unknown_color_cones[i].point.x;
    new_cd_message.other_cones[i].pos.y = msg.unknown_color_cones[i].point.y;
    new_cd_message.other_cones[i].type = utfr_msgs::msg::Cone::UNKNOWN;
  } 
  RCLCPP_INFO(this->get_logger(),"coneDetectionsCB: Published Cone Detections");
  cone_detections_publisher_->publish(new_cd_message);
}

void UTFRSimBridge::teensyCB(
    const utfr_msgs::msg::Teensy& msg){
  //current_lv_sensors_ = *msg;
  RCLCPP_INFO(this->get_logger(), "teensyCB: Throttle = %f", msg.throttle_pv);
}

}
}