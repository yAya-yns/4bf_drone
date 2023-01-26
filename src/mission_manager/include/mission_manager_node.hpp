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
* file: mission_manager_node.hpp
* auth: Kelvin Cui
* desc: mission manager node class header
*/

// ROS2 Requirements
#include <rclcpp/rclcpp.hpp>

// System Requirements
#include <chrono>
#include <functional>
#include <string>

// Message Requirements
#include <utfr_msgs/msg/heartbeat.hpp>
#include <utfr_msgs/msg/system_status.hpp>
#include <utfr_msgs/msg/teensy.hpp>

// UTFR Common Requirements
#include <utfr_common/topics.hpp>

// Library Requirements
#include <heartbeat_monitor/heartbeat_monitor.hpp>

// Misc Requirements:
using std::placeholders::_1; //for std::bind

namespace utfr_dv {
namespace mission_manager{
  
class MissionManagerNode : public rclcpp::Node
{
public:
  /*! Constructor, calls loadParams, initSubscribers initPublishers and 
   *  initTimers.
   */
  MissionManagerNode();

private:
  /*! Initialize and load params from config.yaml:
   * 
   *  publish_rate : double : 
   *    Publish rate in [ms] for come_map_publisher_
   */
  void initParams();

  /*! Initialize all Libraries, including:
   *  - HearbeatMonitor
   */
  void initLibraries();

  /*! Initialize Subscribers:
   *
   *  teensy_subscriber_:
   *    msg: utfr_msgs::Teensy, topic: kTeensy
   */
  void initSubscribers();

  /*! Initialize Publishers:
   *  
   *  system_status_publisher_:
   *    msg: utfr_msgs::SystemStatus, topic: kSystemStatus
   */
  void initPublishers();

  /*! Initialize Timers:
   * 
   *  main_timer_: publishes system status at publish_rate_;
        callback: timerCB() 
   */
  void initTimers();

  /*! Callback function for teensy_subscriber_
   *
   *  @param[in] msg utfr_msgs::msg::Teensy latest teensy message from 
   *  dv controller, including go/nogo status from RES
   */
  void teensyCB(const utfr_msgs::msg::Teensy& msg);

  /*! Callback function all heartbeat subscribers in heartbeat_subscribers_
   *
   *  @param[in] msg utfr_msgs::msg::Heartbeat latest heartbeat for each module,
   *  which get passed onto HeartbeatMonitor::updateHeartbeat().
   */
  void heartbeatCB(const utfr_msgs::msg::Heartbeat& msg);
  
  /*! Main callback function for main_timer_;
   *  Checks heartbeat monitor, mission status, and publishes system status.
   */
  void timerCB();

  // Publishers, Subscribers and Timers
  rclcpp::Subscription<utfr_msgs::msg::Teensy>::SharedPtr 
      teensy_subscriber_;
  std::map<
      std::string, rclcpp::Subscription<utfr_msgs::msg::Heartbeat>::SharedPtr>
      heartbeat_subscribers_;

  rclcpp::Publisher<utfr_msgs::msg::SystemStatus>::SharedPtr 
      system_status_publisher_;
  rclcpp::TimerBase::SharedPtr main_timer_;


  //Libraries
  HeartbeatMonitorUPtr heartbeat_monitor_{nullptr};

  // Parameters
  double publish_rate_;
  double heartbeat_tolerance_;
  std::vector<std::string> heartbeat_modules_;

  // Callback Variables
  utfr_msgs::msg::SystemStatus::SharedPtr system_status_{nullptr};

  bool gonogo_ = false;

  std::unordered_map<std::string, std::string> heartbeat_topics_map_{
    {"control_systems", topics::kControlSystemsHeartbeat},
    {"mapping", topics::kMappingHeartbeat},
    {"navigation", topics::kNavigationHeartbeat},
    {"perception", topics::kPerceptionHeartbeat},
    {"state_estimation", topics::kStateEstimationHeartbeat},
  };
};

} //namespace mission_manager
} //namespace utfr_dv