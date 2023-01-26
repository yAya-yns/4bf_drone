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
* file: controller_pub_node.cpp
* auth: Youssef Elhadad
* desc: controller publisher node class for ros2 template
*/
#include <control_systems_node.hpp>

namespace utfr_dv {
namespace control_systems {
  
ControlSystemsNode::ControlSystemsNode():Node("control_systems_node"){
    this->initParams();
    this->initPublishers();
    this->initController();
    this->initSubscribers();
    this->initTimers();
    this->initHeartbeat();
}

void ControlSystemsNode::initParams(){
  //wheelbase
  // Initialize Params with default values

  this->declare_parameter(
      "pure_pursuit_parameters/wheelbase", 1.0);

  // Load params from parameter server
  wheelbase_ = this->get_parameter(
      "pure_pursuit_parameters/wheelbase").as_double();
  
  //lookahead_distance_scaling_factor
  this->declare_parameter(
      "pure_pursuit_parameters/lookahead_distance_scaling_factor", 1.0);

  lookahead_distance_scaling_factor_ = this->get_parameter(
      "pure_pursuit_parameters/lookahead_distance_scaling_factor").as_double();

  //steering_controller_params
  std::vector<double> default_pid = {0, 0, 0, 0};
  this->declare_parameter(
      "pid_parameters/steering_controller_params", default_pid);

  str_ctrl_params_ = this->get_parameter(
      "pid_parameters/steering_controller_params").as_double_array();

  //velocity_controller_params
  this->declare_parameter(
      "pid_parameters/velocity_controller_params", default_pid);

  vel_ctrl_params_ = this->get_parameter(
      "pid_parameters/velocity_controller_params").as_double_array();
  
  //update_rate_
  this->declare_parameter(
      "update_rate", 1000.0);

  update_rate_ = this->get_parameter(
      "update_rate").as_double();

  current_steering_angle = 0;
}

void ControlSystemsNode::initPublishers(){
  control_cmd_publisher_ = this->create_publisher<utfr_msgs::msg::ControlCmd>(
      topics::kControlCmd,1);
  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kControlSystemsHeartbeat,1);
  
}

void ControlSystemsNode::initTimers(){
  main_timer_ = this->create_wall_timer(
      std::chrono::duration<double,std::milli>(this->update_rate_),
      std::bind(&ControlSystemsNode::timerCB, this));
}

void ControlSystemsNode::initController(){
  pure_pursuit_ = std::make_unique<PurePursuitController>();
  steering_pid_ = std::make_unique<PIDController>();
  velocity_pid_ = std::make_unique<PIDController>(); 
  pure_pursuit_->initController(
      wheelbase_, lookahead_distance_scaling_factor_);

  steering_pid_->initController(str_ctrl_params_, "steering controller");
  velocity_pid_->initController(vel_ctrl_params_, "velocity controller");
}

void ControlSystemsNode::initHeartbeat(){
  heartbeat_.module.data = "control_systems";
  heartbeat_.update_rate = update_rate_;
}

void ControlSystemsNode::publishHeartbeat(const int status){
  heartbeat_.status = status;
  heartbeat_.header.stamp = this->get_clock()->now();
  heartbeat_publisher_->publish(heartbeat_);
}

void ControlSystemsNode::timerCB(){
  const std::string function_name{"timerCB: "};

  double dt = 1; //TODO : use ROS Time
  utfr_msgs::msg::ControlCmd current_control_cmd;

  double steering_setpoint = 
      pure_pursuit_->getSteeringAngle(current_target_state_, 
                                      current_ego_state_);

  double steering_cmd = 
      steering_pid_->getCommand(steering_setpoint,                         //SP
                                current_steering_angle,          //PV
                                dt);

  
  int radius = 10;
  if(current_ego_state_.pos.x < (current_target_state_.trajectory.pos.x+radius) &&
  current_ego_state_.pos.x > (current_target_state_.trajectory.pos.x-radius)&&
  current_ego_state_.pos.y < (current_target_state_.trajectory.pos.y+radius) &&
  current_ego_state_.pos.y > (current_target_state_.trajectory.pos.y-radius))
    current_control_cmd.brk_cmd = 1;
  else{
    if(sqrt(pow(current_ego_state_.vel.x,2)+pow(current_ego_state_.vel.y,2)) < 10){
      current_control_cmd.thr_cmd = 0.05;
    }
    else
      current_control_cmd.thr_cmd = 0;
  }
  
  if(isnan(current_ego_state_.pos.x)){
    current_control_cmd.thr_cmd = 0.1;
    current_control_cmd.str_cmd = 0; 
  }
  else {
    current_control_cmd.str_cmd = steering_setpoint;
  }
  current_control_cmd.acceleration = 1.0;
  control_cmd_publisher_->publish(current_control_cmd);

  //current_control_cmd.brk_cmd = ;
  current_steering_angle = current_control_cmd.str_cmd ;
  RCLCPP_INFO(this->get_logger(),"%sCurrent y Position = %f  Steering point = %f steering_cmd = %f",
    function_name, current_ego_state_.pos.y, current_control_cmd.str_cmd, steering_cmd);

  counter_++;

  //TODO - change status based on operation
  int status = utfr_msgs::msg::Heartbeat::ACTIVE;

  this->publishHeartbeat(status);
}

void ControlSystemsNode::initSubscribers(){
  target_state_subscriber_ = 
    this->create_subscription<utfr_msgs::msg::TargetState>(
        topics::kTargetState, 1, 
        std::bind(&ControlSystemsNode::targetStateCB, this, _1));

  ego_state_subscriber_ = 
    this->create_subscription<utfr_msgs::msg::EgoState>(
          topics::kEgoState, 1, 
          std::bind(&ControlSystemsNode::egoStateCB, this, _1));

  teensy_subscriber_ = 
    this->create_subscription<utfr_msgs::msg::Teensy>(
          topics::kTeensy, 1, 
          std::bind(&ControlSystemsNode::teensyCB, this, _1));
}

void ControlSystemsNode::targetStateCB(
    const utfr_msgs::msg::TargetState& msg){
  current_target_state_ = msg;
  RCLCPP_INFO(this->get_logger(), "Velocity = %f", msg.trajectory.velocity);
}

void ControlSystemsNode::egoStateCB(
    const utfr_msgs::msg::EgoState& msg){
  current_ego_state_ = msg;
  RCLCPP_INFO(this->get_logger(), "egoStateCB: Pos x = %f", msg.pos.x);
}

void ControlSystemsNode::teensyCB(
    const utfr_msgs::msg::Teensy& msg){
  //current_lv_sensors_ = msg;
  RCLCPP_INFO(this->get_logger(), "teensyCB: Throttle = %f", msg.throttle_pv);
}

}
} 