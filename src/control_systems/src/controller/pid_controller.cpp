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
* file: pid.cpp
* auth: Youssef Elhadad
* desc: pid controller base class
*
*/

#include "controller/pid_controller.hpp"

namespace utfr_dv {
namespace control_systems{

  void PIDController::initController(
      const std::vector<double>& params,
      const std::string& name){
    name_ = name;
    this->setParams(params);
    this->resetVariables();
  }

  void PIDController::setParams(const std::vector<double>& params){
    const std::string function_name{
        "PIDController:setParams" + name_ + "Controller"};
    std::string key = "";
    try{
      p_gain_ = params.at(0);
      if (p_gain_ < 0){
        throw std::invalid_argument("p_gain < 0");
      }
      i_gain_ = params.at(1);
      if (i_gain_ < 0){
        throw std::invalid_argument("i_gain < 0");
      }
      d_gain_ = params.at(2);
      if (d_gain_ < 0){
        throw std::invalid_argument("d_gain < 0");
      }

      output_lim_ = params.at(3);
      if (output_lim_ < 0){
        throw std::invalid_argument("output_lim < 0");
      }
    }
    catch(const std::out_of_range& e){
      std::string error = function_name;
      error += e.what();
      error += " '" + key + "', please verify controller config.yaml";
      throw std::out_of_range(error);
    }
    catch(const std::invalid_argument& e){
      std::string error = function_name;
      error += e.what();
      error += ", please verify controller config.yaml";
      throw std::invalid_argument(error);
    } 
  }

  double PIDController::getCommand(
      double pv, double sp, double dt){
    double error = pv - sp;

    error_sum_ += error * dt; // compute integral

    double error_rate = (error - error_prev_) / dt; // compute derivative
    double cmd = p_gain_ * error + i_gain_ * error_sum_ + d_gain_ * error_rate; // PID output  

    error_prev_ = error; //remember current error 
    //double cmd = pv;

    //Make sure it is between -output_lim_ and output_lim_
    cmd = std::clamp(cmd, -output_lim_, output_lim_);
    return cmd;
  }

  void PIDController::resetVariables(){
    error_sum_ = 0;
    error_prev_ = 0;
  }


} // namespace control_systems
} // namespace utfr_dv
