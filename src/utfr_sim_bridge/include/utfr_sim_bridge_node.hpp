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
* file: utfr_sim_bridge_node.hpp
* auth: Youssef Elhadad
* desc: controllers publisher and subscribers node header (ros 2)
*/
#pragma once

//System Requirements
#include <chrono>
#include <functional>
#include <string>
#include <chrono>
#include <cstdlib>
#include <memory>

// ROS2 Requirements
#include <rclcpp/rclcpp.hpp>

//UTFR Common Requirements
#include <utfr_common/math.hpp>
#include <utfr_common/topics.hpp>

//Message Requirements
#include <std_msgs/msg/string.hpp>
#include <utfr_msgs/msg/ego_state.hpp>
#include <utfr_msgs/msg/target_state.hpp>
#include <utfr_msgs/msg/teensy.hpp>
#include <utfr_msgs/msg/control_cmd.hpp>
#include <utfr_msgs/msg/heartbeat.hpp>
#include <utfr_msgs/msg/cone_detections.hpp>
#include <utfr_msgs/msg/cone.hpp>

//Sim Requirements
#include <eufs_msgs/srv/set_can_state.hpp>
#include <eufs_msgs/msg/car_state.hpp>
#include <eufs_msgs/msg/can_state.hpp>
#include <eufs_msgs/msg/cone_array_with_covariance.hpp>
#include <eufs_msgs/msg/vehicle_commands.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

// Misc Requirements:
using std::placeholders::_1; //for std::bind

using namespace std::chrono_literals;

namespace utfr_dv {
namespace utfr_sim_bridge {

class UTFRSimBridge : public rclcpp::Node{
 public:

  UTFRSimBridge();

 private:

  /*! Initialize Publishers:
   *
   *  control_cmd_publisher_ : ackermann_msgs::msg::AckermannDriveStamped, topic: kControlCmd
   *    utfr_msgs to be published
   */
  void initPublishers();
  
  /*! Initialize Subscribers:
   *
   *  target_state_subscriber_:
   *  msg: utfr_msgs::TargetState, topic: kTargetState, 
   *  callback: targetStateCB
   * 
   *  ego_state_subscriber_:
   *  msg: utfr_msgs::EgoState, topic: kEgoState, 
   *  callback: egoStateCB
   */ 
  void initSubscribers();

  /*! Initialize Client:
   *
   *  mission_client_:
   *  client: eufs_msgs::srv::SetCanState, topic: kTargetState, 
   *  callback: targetStateCB
   * 
   *  ego_state_subscriber_:
   *  msg: utfr_msgs::EgoState, topic: kEgoState, 
   *  callback: egoStateCB
   */ 
  void initClients();
  
  /*! Send Heartbeat on every timer loop.
   *
   *  @param[in] status current module status, using Heartbeat status enum.
   */
  void publishHeartbeat(const int status);

  /*! Target State callback function for target_state_subscriber_
   *
   *  @param[in] msg utfr_msgs::TargetState incoming message
   */
  void controlCmdCB(const utfr_msgs::msg::ControlCmd& msg);

  /*! Ego State callback function for ego_state_subscriber_
   *
   *  @param[in] msg utfr_msgs::EgoState incoming message
   */
  void egoStateCB(const eufs_msgs::msg::CarState& msg);

  /*! Cone Detections callback function for cone_detections_subscriber_
   *
   *  @param[in] msg utfr_msgs::ConeDetections incoming message
   */
  void coneDetectionsCB(const eufs_msgs::msg::ConeArrayWithCovariance& msg);

  /*! Ego State callback function for teensy_subscriber_
   *
   *  @param[in] msg utfr_msgs::Teensy incoming message
   */
  void teensyCB(const utfr_msgs::msg::Teensy& msg);


  //Publisher

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr 
      control_cmd_publish_;

  rclcpp::Publisher<utfr_msgs::msg::EgoState>::SharedPtr 
      state_estimation_publisher_;
    
  rclcpp::Publisher<utfr_msgs::msg::ConeDetections>::SharedPtr 
      cone_detections_publisher_;

  rclcpp::Publisher<utfr_msgs::msg::TargetState>::SharedPtr 
      target_state_publisher_;

  //Subscriber
  rclcpp::Subscription<utfr_msgs::msg::ControlCmd>::SharedPtr  
      control_cmd_subscriber_;

  rclcpp::Subscription<eufs_msgs::msg::CarState>::SharedPtr  
      ego_state_subscriber_;

  rclcpp::Subscription<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr  
      cone_detections_subscriber_;

  rclcpp::Subscription<utfr_msgs::msg::Teensy>::SharedPtr  
      teensy_subscriber_;

  //CLient 
  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr 
      mission_client_;
  
};

} // namespace control_systems
} // namespace utfr_dv
