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
* file: mapping_node.hpp
* auth: Kelvin Cui
* desc: mapping node class header
*/

// ROS2 Requirements
#include <rclcpp/rclcpp.hpp>

// System Requirements
#include <chrono>
#include <functional>
#include <string>

// Transform Requirements
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Message Requirements
#include <utfr_msgs/msg/cone_detections.hpp>
#include <utfr_msgs/msg/cone_map.hpp>
#include <utfr_msgs/msg/heartbeat.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

// UTFR Common Requirements
#include <utfr_common/math.hpp>
#include <utfr_common/topics.hpp>
#include <utfr_common/frames.hpp>

// Misc Requirements:
#include <fusion/fusion.hpp>
using std::placeholders::_1; //for std::bind

namespace utfr_dv {
namespace mapping{
  
class MappingNode : public rclcpp::Node
{
public:
  /*! Constructor, calls loadParams, initSubscribers initPublishers and 
   *  initTimers.
   */
  MappingNode();

private:
  /*! Initialize and load params from config.yaml:
   * 
   *  publish_rate : double : 
   *    Publish rate in [ms] for come_map_publisher_
   */
  void initParams();

  /*! Initialize Subscribers:
   *
   *  cone_detections_subscriber_:
   *    msg: utfr_msgs::ConeDetections, topic: kConeDetections
   *   
   *  lidar_subscriber_:
   *    msg: sensor_msgs:laser_scan, topic: kLidarDetections
   */
  void initSubscribers();

  /*! Initialize Publishers:
   *  
   *  come_map_publisher_:
   *    msg: utfr_msgs::ConeMap, topic: kConeMap
   */
  void initPublishers();

  /*! Initialize Transform Tree.
   *  Create the pointers to the tf buffer and tf listener objects
   */
  void initTransforms();

  /*! Initialize the fusion module.
   */
  void initFusion();

  /*! Setup Heartbeat message with appropriate module name and update rate.
   */
  void initHeartbeat();

  /*! Initialize Timers:
   * 
   *  main_timer_: publishes cone_map at publish_rate
   *    callback: timerCB() 
   */
  void initTimers();

  /*! Send Heartbeat on every timer loop.
   *
   *  @param[in] status current module status, using Heartbeat status enum.
   */
  void publishHeartbeat(const int status);

  /*! Callback function for cone_detection_subscriber_
   *
   *  @param[in] msg utfr_msgs::msg::ConeDetections latest cone detections from 
   *  perception
   */
  void coneDetectionsCB(const utfr_msgs::msg::ConeDetections& msg);

  /*! Callback function for lidar_detection_subscriber_
   *
   *  @param[in] msg sensor_msgs::msg::LaserScan latest lidar scan
   */
  void lidarDetectionsCB(const sensor_msgs::msg::LaserScan& msg);

  /*! Publish latest current_map_
   *  Resets all new data flags to wait for new data
   */
  void publishMap();

  /*! Update map given new data
   */
  void updateMap();
  
  /*! Main callback function for main_timer_;
   *  Publishes latest version of cone map
   */
  void timerCB();

  // Publishers, Subscribers and Timers
  rclcpp::Subscription<utfr_msgs::msg::ConeDetections>::SharedPtr 
      cone_detections_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr 
      lidar_detections_subscriber_;
  rclcpp::Publisher<utfr_msgs::msg::ConeMap>::SharedPtr 
      cone_map_publisher_;
  rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr
      heartbeat_publisher_;
  rclcpp::TimerBase::SharedPtr main_timer_;

  // Transforms
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Fusion
  FusionUPtr fusion_{nullptr};

  // Parameters
  double update_rate_;

  // Callback Variables
  utfr_msgs::msg::ConeDetections::SharedPtr current_camera_{nullptr};
  std::shared_ptr<std::vector<utfr_msgs::msg::Cone>> current_lidar_{nullptr};
  utfr_msgs::msg::ConeMap::SharedPtr current_map_{nullptr};
  utfr_msgs::msg::Heartbeat heartbeat_;

  bool new_camera_detections_ = false;
  bool new_lidar_detections_ = false;

};

} //namespace mapping
} //namespace utfr_dv