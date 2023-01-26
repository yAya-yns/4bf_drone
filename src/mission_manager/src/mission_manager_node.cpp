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
* file: mission_manager_node.cpp
* auth: Kelvin Cui
* desc: mapping node class
*/

#include <mission_manager_node.hpp>

namespace utfr_dv {
namespace mission_manager{

MissionManagerNode::MissionManagerNode():Node("mission_manager_node")
  {
   this->initParams();
   this->initLibraries();
   this->initSubscribers();
   this->initPublishers();
   this->initTimers();
  }

void MissionManagerNode::initParams(){

  std::vector<std::string> default_modules = 
      {"control_systems",
       "mapping",
       "navigation",
       "perception",
       "state_estimation"};

  // Initialize Params with default values
  this->declare_parameter("update_rate", 10.0);
  this->declare_parameter("heartbeat_tolerance", 1.5);
  this->declare_parameter("heartbeat_modules", default_modules);

  // Load params from parameter server
  publish_rate_ = 
      this->get_parameter("update_rate").as_double();

  heartbeat_tolerance_ = 
      this->get_parameter("heartbeat_tolerance").as_double();

  heartbeat_modules_=
      this->get_parameter("heartbeat_modules").as_string_array();
}

void MissionManagerNode::initLibraries(){
  heartbeat_monitor_ = std::make_unique<HeartbeatMonitor>(
       heartbeat_modules_, this->get_clock()->now(), heartbeat_tolerance_);
}

void MissionManagerNode::initSubscribers(){

  const std::string function_name{"MissionManagerNode::initSubscribers: "};

  for (const auto& module_name: heartbeat_modules_) {

    auto search = heartbeat_topics_map_.find(module_name);

    if (search == heartbeat_topics_map_.end()){ //Module not found :
    RCLCPP_ERROR(this->get_logger(), "%s Module %s topic not in map", 
        function_name.c_str(), module_name.c_str()); 
    continue;
    }

    std::string topic = heartbeat_topics_map_[module_name]; 

    heartbeat_subscribers_[module_name] =
       this->create_subscription<utfr_msgs::msg::Heartbeat>(
            topic, 10, 
            std::bind(&MissionManagerNode::heartbeatCB, this, _1));
  }
  
  teensy_subscriber_ = this->create_subscription<utfr_msgs::msg::Teensy>(
            topics::kTeensy, 10, 
            std::bind(&MissionManagerNode::teensyCB, this, _1));
}

void MissionManagerNode::initPublishers(){
  system_status_publisher_ = 
      this->create_publisher<utfr_msgs::msg::SystemStatus>(
          topics::kSystemStatus,10);
}

void MissionManagerNode::initTimers(){
  main_timer_ = this->create_wall_timer(
      std::chrono::duration<double,std::milli>(publish_rate_),
      std::bind(&MissionManagerNode::timerCB, this));
}

void MissionManagerNode::teensyCB(const utfr_msgs::msg::Teensy& msg){
  gonogo_ = msg.gonogo;
}

void MissionManagerNode::heartbeatCB(const utfr_msgs::msg::Heartbeat& msg){
  heartbeat_monitor_->updateHeartbeat(msg, this->get_clock()->now());
}

void MissionManagerNode::timerCB(){

  const std::string function_name {"MissionManagerNode::timerCB:"};

  if (system_status_ == nullptr){
    //Initialize System Status Message Pointer
    utfr_msgs::msg::SystemStatus template_status;
    template_status.status = utfr_msgs::msg::SystemStatus::OFF;
    system_status_ = 
        std::make_shared<utfr_msgs::msg::SystemStatus>(template_status);
  }

  system_status_->header.stamp = this->get_clock()->now();

  bool heartbeat_status = 
      heartbeat_monitor_->verifyHeartbeats(this->get_clock()->now());

  //AS Status State Machine
  switch(system_status_->status) {
    case utfr_msgs::msg::SystemStatus::OFF:{

      gonogo_ = false; //debounce gonogo

      if (heartbeat_status){
        // All critical modules loaded
        system_status_->status = utfr_msgs::msg::SystemStatus::READY;
      }
      break;

    }
    case utfr_msgs::msg::SystemStatus::READY:{

      if (!heartbeat_status) { //Heartbeats failed after loading correctly
        system_status_->status = utfr_msgs::msg::SystemStatus::EMERGENCY;
      }

      if (gonogo_) { //RES Go recieved
        //TODO - launch mission
        system_status_->status = utfr_msgs::msg::SystemStatus::DRIVING;
      }
      break;
    }
    case utfr_msgs::msg::SystemStatus::DRIVING:{
      if (!heartbeat_status) { //Heartbeats failed after loading correctly
        system_status_->status = utfr_msgs::msg::SystemStatus::EMERGENCY;
      }

      //TODO - switch to finished case when mission complete
      break;

    }
    case utfr_msgs::msg::SystemStatus::EMERGENCY:{
      // TODO - shutdown system appropriately
      break;

    }
    case utfr_msgs::msg::SystemStatus::FINISHED:{
      // TODO - shutdown system appropriately
      break;

    }
  }

  system_status_publisher_->publish(*system_status_);
}


} //namespace mission_manager
} //namespace utfr_dv