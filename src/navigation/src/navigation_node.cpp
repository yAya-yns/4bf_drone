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
* file: navigation_node.cpp
* auth: Daniel Asadi
* desc: Navigation node class
*/

#include <navigation_node.hpp>

namespace utfr_dv {
namespace navigation {

    NavigationNode::NavigationNode() : Node("navigation_node")
    {
      this->initParams();
      if (!this->initPlanner())
      {
        return;
      }
      this->initSubscribers();
      this->initPublishers();
      this->initTimers();
      this->initHeartbeat();
    }

    void NavigationNode::initParams() // TODO: Add new params
    {
      // Initialize Params with default values
      this->declare_parameter("update_rate", 33.33);
      this->declare_parameter("mission_path", "/paths/accel_coords.csv");
      this->declare_parameter("target_radius", 1);

      // Load params from parameter server
      update_rate_ =
          this->get_parameter("update_rate").get_parameter_value().get<double>();
      mission_path_file_ =
          this->get_parameter("mission_path").get_parameter_value().get<std::string>();
      target_radius_ =
          this->get_parameter("target_radius").get_parameter_value().get<double>();
    }

    void NavigationNode::initSubscribers()
    {
      ego_state_subscriber_ =
          this->create_subscription<utfr_msgs::msg::EgoState>(
              topics::kEgoState, 10,
              std::bind(&NavigationNode::egoStateCB, this, _1));

      cone_map_subscriber_ =
          this->create_subscription<utfr_msgs::msg::ConeMap>(
              topics::kConeMap, 10,
              std::bind(&NavigationNode::coneMapCB, this, _1));
    }

    void NavigationNode::initPublishers()
    {
      target_state_publisher_ =
          this->create_publisher<utfr_msgs::msg::TargetState>(topics::kTargetState, 10);
      heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
          topics::kNavigationHeartbeat,1);
    }

    void NavigationNode::initTimers()
    {
      main_timer_ = this->create_wall_timer(
          std::chrono::duration<double, std::milli>(update_rate_),
          std::bind(&NavigationNode::timerCB, this));

    }

    bool NavigationNode::initPlanner()
    {
      const std::string function_name{"NavigationNode::initPlanner:"};
      try
      {
        trajectory_path_ = this->readCSV(mission_path_file_);
      }
      catch (const std::runtime_error &e)
      {
        RCLCPP_ERROR(this->get_logger(),
                     " %s NavigationNodelet::initPlanner: Caught runtime err from readCSV: %s from %s",
                     function_name.c_str(), e.what(), mission_path_file_.c_str());

        return false;
      }

      if (!verifyTrajectoryPath(trajectory_path_))
      {
        return false;
      }

      trajectory_point_count_ = trajectory_path_.size();
      end_state_->trajectory = trajectory_path_.back();

      trajectory_idx_ = 1;
      curr_target_->trajectory = trajectory_path_[trajectory_idx_];

      mission_ready_ = true;
      return true;
    }

    void NavigationNode::egoStateCB(const utfr_msgs::msg::EgoState &msg)
    {
      utfr_msgs::msg::EgoState curr_ego_state_ = msg;
    }

    void NavigationNode::coneMapCB(const utfr_msgs::msg::ConeMap &msg)
    {
      utfr_msgs::msg::ConeMap curr_cone_map_ = msg;
    }

    void NavigationNode::initHeartbeat(){
      heartbeat_.module.data = "navigation";
      heartbeat_.update_rate = update_rate_;
    }

    void NavigationNode::publishHeartbeat(const int status){
      heartbeat_.status = status;
      heartbeat_.header.stamp = this->get_clock()->now();
      heartbeat_publisher_->publish(heartbeat_);
    }

    void NavigationNode::timerCB()
    {
      const std::string function_name{"NavigationNode::timerCB:"};

      if (!mission_ready_)
      {
        RCLCPP_WARN(this->get_logger(),
                    "%s mission not yet ready!", function_name.c_str());
        return;
      }

      if (curr_ego_state_ == nullptr)
      {
        RCLCPP_WARN(this->get_logger(),
                    "%s ego state not yet ready!", function_name.c_str());
        return;
      }

      if (curr_cone_map_ == nullptr)
      {
        RCLCPP_WARN(this->get_logger(),
                    "%s cone map not yet ready!", function_name.c_str());
        return;
      }

      // TODO - Change status to AS Status
      if (curr_target_->trajectory.type == utfr_msgs::msg::TrajectoryPoint::ACTIVE)
      {
        getNextTrajectory();
      }

      else
      {
        RCLCPP_INFO(this->get_logger(),
                    "%s mission complete!", function_name.c_str());
      }

      target_state_publisher_->publish(*curr_target_);

      //TODO - change status based on operation
      this->publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);
    }

    void NavigationNode::getNextTrajectory()
    {
      const std::string function_name{"NavigationNodelet::getNextTrajectory"};

      // Check current distance to next point
      if (util::euclidianDistance2D(
              curr_ego_state_->pos,
              curr_target_->trajectory.pos) < target_radius_)
      {

        trajectory_idx_++;
        if (trajectory_idx_ > trajectory_point_count_)
        {
          curr_target_ = end_state_;
          return;
        }
        curr_target_->trajectory = trajectory_path_[trajectory_idx_];
      }
    }

    bool NavigationNode::verifyTrajectoryPath(
        const std::vector<utfr_msgs::msg::TrajectoryPoint> &path)
    {

      const std::string function_name{"NavigationNodelet::verifyTrajectoryPath: "};

      // Make sure at least 3 points exist
      if (path.size() < 3)
      {
        RCLCPP_ERROR(this->get_logger(),
                     " %s Path is less than 3 points!", function_name.c_str());
        return false;
      }

      // Make sure the first and last points are labeled correctly
      if (path.front().type != utfr_msgs::msg::TrajectoryPoint::START)
      {
        RCLCPP_ERROR(this->get_logger(),
                     " %s First point is not starting point!", function_name.c_str());
        return false;
      }
      if (path.back().type != utfr_msgs::msg::TrajectoryPoint::FINISH)
      {
        RCLCPP_ERROR(this->get_logger(),
                     " %s Final point is not ending point!", function_name.c_str());
        return false;
      }

      if (path.back().velocity > 0)
      {
        RCLCPP_ERROR(this->get_logger(),
                     " %s Final point has non-zero velocity!", function_name.c_str());
        return false;
      }

      return true;
    }

    std::vector<utfr_msgs::msg::TrajectoryPoint> NavigationNode::readCSV(
        const std::string &filepath)
    {

      const std::string function_name{"NavigationNodelet::readCSV: "};

      // Init vars
      std::string dir_path;
      utfr_msgs::msg::TrajectoryPoint point;
      std::vector<utfr_msgs::msg::TrajectoryPoint> points_list;
      geometry_msgs::msg::Vector3 target_pos;

      // Find current ros directory:
      try
      {
        dir_path = ament_index_cpp::get_package_share_directory("navigation");
      }
      catch (const ament_index_cpp::PackageNotFoundError &e)
      {
        RCLCPP_ERROR(this->get_logger(),
                     " %s Error getting file path!", function_name.c_str());
      }

      // Create an input filestream
      std::ifstream myFile(dir_path + filepath);
      if (!myFile.is_open())
      {
        throw std::runtime_error("Could not open file");
      }
      std::string line;

      // Extract the first line in the file and ignore (column titles)
      if (myFile.good())
      {
        std::getline(myFile, line);
      }

      // Read data, line by line
      while (std::getline(myFile, line))
      {
        std::stringstream ss(line);

        std::vector<std::string> results;
        boost::split(results, ss.str(), [](char c)
                     { return c == ','; });

        if (results.at(1) == "START")
        {
          point.type = utfr_msgs::msg::TrajectoryPoint::START;
        }
        else if (results.at(1) == "INTERMEDIATE")
        {
          point.type = utfr_msgs::msg::TrajectoryPoint::ACTIVE;
        }
        else if (results.at(1) == "FINISH")
        {
          point.type = utfr_msgs::msg::TrajectoryPoint::FINISH;
        }
        else
        {
          throw std::runtime_error("Invalid Point type");
        }

        target_pos.x = std::stod(results.at(2));
        target_pos.y = std::stod(results.at(3));
        target_pos.z = 0;
        point.velocity = std::stod(results.at(4));
        point.pos = target_pos;
        points_list.push_back(point);
        if (ss.peek() == ',')
          ss.ignore();
      }

      // Close file
      myFile.close();
      return points_list;
    }

} // namespace navigation
} // namespace utfr_dv