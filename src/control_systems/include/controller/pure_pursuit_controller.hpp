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
* file: pure_pursuit.hpp
* auth: Youssef Elhadad
* desc: pure pursuit lateral controller header
*/
#pragma once

//System Requirements
#include <math.h>
//System Requirements
#include <chrono>
#include <functional>
#include <string>

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

#define PI 3.1415926535

namespace utfr_dv {
namespace control_systems {

class PurePursuitController{
 public:
  /*! Initialize controller with car parameters.
   *
   *  @param[in] wheelbase double of vehicle wheelbase
   *  @param[in] ld_sf double of lookhead distance scaling factor
   */
  void initController(double wheelbase, double ld_sf);

  /*! Calculate Steering angle using pure pursuit algorithim.
   *
   *  @param[in] target TargetState of closest Trajectory Point
   *  @param[in] ego EgoState of latest ego state
   *  @returns double of steering angle in [deg].
   * 
   */
  double getSteeringAngle(
      const utfr_msgs::msg::TargetState& target, 
      const utfr_msgs::msg::EgoState& ego);

 private:

  double wheelbase_;
  double lookahead_distance_scaling_factor_;
};

using PurePursuitControllerUPtr = std::unique_ptr<PurePursuitController>;
using PurePursuitControllerSPtr = std::shared_ptr<PurePursuitController>;

} // namespace control_systems
} // namespace utfr_dv