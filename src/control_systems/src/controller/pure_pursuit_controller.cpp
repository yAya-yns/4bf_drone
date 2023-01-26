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
* file: pure_pursuit.cpp
* auth: Youssef Elhadad
* desc: pure pursuit lateral controller
*
*/

#include "controller/pure_pursuit_controller.hpp"

namespace utfr_dv {
namespace control_systems {

  void PurePursuitController::initController(double wheelbase, double ld_sf){
    wheelbase_ = wheelbase;
    lookahead_distance_scaling_factor_ = ld_sf;
  }

  double PurePursuitController::getSteeringAngle(
      const utfr_msgs::msg::TargetState& target, 
      const utfr_msgs::msg::EgoState& ego){

    geometry_msgs::msg::Vector3 ego_pos = ego.pos;
    geometry_msgs::msg::Vector3 target_pos = target.trajectory.pos;

    double ego_velocity = sqrt(pow(ego.vel.x,2)+pow(ego.vel.y,2));
    
    //Check if velocity is zero or null
    if(isnan(ego_velocity) || ego_velocity == 0 || isnan(target_pos.x)) {
      return 0;
    }


    //PurePursuit Algorithim

    //Step 1: Calculate the pos at the rear of the car
    geometry_msgs::msg::Vector3 rear_pos;
    //using rot from North
    rear_pos.x = ego_pos.x - cos((ego_pos.z))*wheelbase_/2;
    rear_pos.y = ego_pos.y - sin((ego_pos.z))*wheelbase_/2;
    

    //Step 2: find vector from rear to target 
    geometry_msgs::msg::Vector3 rear_to_target;
    geometry_msgs::msg::Vector3 rear_to_center;
    rear_to_target.x = target_pos.x - rear_pos.x;
    rear_to_target.y = target_pos.y - rear_pos.y;
    rear_to_center.x = ego_pos.x - rear_pos.x;
    rear_to_center.y = ego_pos.y - rear_pos.y;
  
    //Step 3: Find dot and cross product  
    double dot_product;
    double cross_product;

    dot_product = (rear_to_target.x*rear_to_center.x + 
                            rear_to_target.y*rear_to_center.y);

    cross_product = (rear_to_center.x*rear_to_target.y - 
      rear_to_target.x*rear_to_center.y);

    double lookahead_distance_from_center = sqrt(pow((target_pos.x - ego_pos.x),2)+
      pow((target_pos.y - ego_pos.y),2));

    double lookahead_distance_from_rear = sqrt(pow(rear_to_target.x,2)+
      pow(rear_to_target.y,2));

    //Step 4: Calculate alpha 
    double alpha;
    if(isnan(dot_product) && (lookahead_distance_from_rear < 
      lookahead_distance_from_center))
      return 1;
    else if(isnan(dot_product) && (lookahead_distance_from_rear > 
      lookahead_distance_from_center))
      return 0;

    alpha = acos(dot_product/(lookahead_distance_from_rear*wheelbase_/2));
    alpha = cross_product<0 ? -alpha:alpha;
    double steering_angle;
    if(alpha>PI-0.0005 && alpha<PI+0.0005){
      alpha=PI/2;
      steering_angle = atan((2 * wheelbase_ * sin(alpha))
                      /lookahead_distance_from_rear);
    }else{
      steering_angle = atan((2 * wheelbase_ * sin(alpha))
                      /lookahead_distance_from_rear);
    }
    double result = round((utfr_dv::util::radToDeg(steering_angle)/90)*100)/100;
    return std::clamp(result,-1.0,1.0);
  }
} // namespace control_systems
} // namespace utfr_dv