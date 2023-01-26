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
* file: control_systems_node.hpp
* auth: Youssef Elhadad
* desc: controllers publisher and subscribers node header (ros 2)
*/
#pragma once

#include <controller/pure_pursuit_controller.hpp>
#include <controller/pid_controller.hpp>

//System Requirements
#include <chrono>
#include <functional>
#include <string>
#include <map>


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


// Misc Requirements
using std::placeholders::_1; //for std::bind

namespace utfr_dv {
namespace control_systems {

class ControlSystemsNode : public rclcpp::Node{
 public:

  ControlSystemsNode();

 private:

  /*! Load params from config file
   *
   *  wheelbase_ : double :
   *    Length of the car
   * 
   *  lookahead_distance_scaling_factor: double :
   *    contains the lookahead distance scaling factor
   *  
   *  str_ctrl_params_ : std::vector<double> : 
   *    parameters for steering PID controller
   *    
   *  vel_ctrl_params : double :
   *    parameters for velocity PID controller
   * 
   *  update_rate : double : 
   *    Publish rate in [ms] for example_publisher_
   */
  void initParams();

  /*! Initialize Publishers:
   *
   *  control_cmd_publisher_ : utfr_msgs::ControlCmd, topic: kControlCmd
   *    utfr_msgs to be published
   * 
   *  heartbeat_publisher : utfr_msgs::msg::Heartbeat, topic: kControlSystemsHeartbeat
   *    utfr_msgs to be published
   * 
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
   * 
   *  teensy_subscriber_: 
   *  msg: utfr_msgs::msg::Teensy, topic: kTeensy,
   *  callback: teensyCB
   *  
   */ 
  void initSubscribers();

  /*! Initialize Timers:
   *
   *  main_timer_:
   *  Calls callback every update_rate_ seconds
   *  callback: ControlSystemsNode::timerCB
   * 
   */ 
  void initTimers();

  /*! Initialize Pure Pursuit Controller by sending wheelbase and ld_sf params.
   */
  void initController();

  /*! Setup Heartbeat message with appropriate module name and update rate.
   */
  void initHeartbeat();

  /*! Send Heartbeat on every timer loop.
   *
   *  @param[in] status current module status, using Heartbeat status enum.
   */
  void publishHeartbeat(const int status);

  /*! Target State callback function for target_state_subscriber_
   *
   *  @param[in] msg utfr_msgs::TargetState incoming message
   */
  void targetStateCB(const utfr_msgs::msg::TargetState& msg);

  /*! Ego State callback function for ego_state_subscriber_
   *
   *  @param[in] msg utfr_msgs::EgoState incoming message
   */
  void egoStateCB(const utfr_msgs::msg::EgoState& msg);

  /*! Teensy callback function for teensy_subscriber_
   *
   *  @param[in] msg utfr_msgs::Teensy incoming message
   */
  void teensyCB(const utfr_msgs::msg::Teensy& msg);

  /*! Primary callback loop for Controllers.
   *  Calculates steering angle command through pure pursuit controller class.
   *  Propgates velocity target from target_state.
   *  Publishes control commands.
   */
  void timerCB();

  //Publisher
  rclcpp::Publisher<utfr_msgs::msg::ControlCmd>::SharedPtr 
      control_cmd_publisher_;

  rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr
      heartbeat_publisher_;

  PurePursuitControllerUPtr pure_pursuit_{nullptr};
  PIDControllerUPTr steering_pid_{nullptr};
  PIDControllerUPTr velocity_pid_{nullptr};

  //Subscriber
  rclcpp::Subscription<utfr_msgs::msg::TargetState>::SharedPtr  
      target_state_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::EgoState>::SharedPtr  
      ego_state_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::Teensy>::SharedPtr  
      teensy_subscriber_;

  rclcpp::TimerBase::SharedPtr main_timer_;

  //Params
  double update_rate_;
  double wheelbase_;
  double lookahead_distance_scaling_factor_;
  std::vector<double> str_ctrl_params_;
  std::vector<double> vel_ctrl_params_;

  
  //Callback
  utfr_msgs::msg::TargetState current_target_state_;
  utfr_msgs::msg::EgoState current_ego_state_;
  utfr_msgs::msg::Teensy current_teensy_msg_;
  utfr_msgs::msg::Heartbeat heartbeat_;
  double current_steering_angle;
  int counter_;
  
};

} // namespace control_systems
} // namespace utfr_dv