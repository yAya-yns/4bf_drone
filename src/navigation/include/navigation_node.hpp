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
* file: navigation_node.hpp
* auth: Daniel Asadi
* desc: navigation node ros2
*/

// ROS2 Requirements
#include <rclcpp/rclcpp.hpp>

// System Requirements
#include <chrono>
#include <functional>
#include <string>
#include <fstream>
#include <vector>
#include <stdexcept> // std::runtime_error
#include <sstream>   // std::stringstream
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/algorithm/string.hpp>

// Message Requirements
#include <std_msgs/msg/string.hpp>
#include <utfr_msgs/msg/cone_map.hpp>
#include <utfr_msgs/msg/ego_state.hpp>
#include <utfr_msgs/msg/target_state.hpp>
#include <utfr_msgs/msg/trajectory_point.hpp>
#include <utfr_msgs/msg/system_status.hpp>
#include <utfr_msgs/msg/heartbeat.hpp>

// UTFR Common Requirements
#include <utfr_common/math.hpp>
#include <utfr_common/topics.hpp>

// Misc Requirements:
using std::placeholders::_1; // for std::bind

namespace utfr_dv {
namespace navigation {

    class NavigationNode : public rclcpp::Node
    {
    public:
        /*! Constructor, calls loadParams, initPublishers and initTimers.
            */
        NavigationNode();

    private:
        /*! Initialize and load params from config.yaml:
            *
            *  publish_string : std::string :
            *    String to be published along with counter of message number
            *
            *  publish_rate : int :
            *    Publish rate in [ms] for example_publisher_
            */
        void initParams();

        /*! Initialize Subscribers:
            *
            * ego_state_subscriber_:
            * msg: utfr_msgs::msg::EgoState, topic: kEgoState
            *
            * cone_map_subscriber_:
            * msg: utfr_msgs::msg::ConeMap, topic: kConeMap
            */
        void initSubscribers();

        /*! Initialize Publishers:
            *
            * target_state_publisher_:
            * msg: utfr_msgs::msg::TargetState, topic: kTargetState
            */
        void initPublishers();

        /*! Initialize Timers:
        *
        *  main_timer_: publishes cone_map at publish_rate
                callback: timerCB()
        */
        void initTimers();

        /*! Initializes map and mission variables based on parameters.
            *  Sets initial target, as well as mission_ready_ state.
            *
            *  @returns bool true if successfuly initialized
            */
        bool initPlanner();

        /*! Ego State Subscriber Callback for ego_state_subscriber_
            *
            *  @param[in] msg EgoStateConstPtr of incoming ego state message
            */
        void initHeartbeat();
        /*! Send Heartbeat on every timer loop.
            *  @param[in] status current module status, using Heartbeat status enum.
            */
        void publishHeartbeat(const int status);
        /*! Target State callback function for target_state_subscriber_
            *
            *  @param[in] msg utfr_msgs::TargetState incoming message
            * */
        void egoStateCB(const utfr_msgs::msg::EgoState &msg);

        /*! Cone Map Subscriber Callback for cone_map_subscriber_
            *
            *  @param[in] msg ConeMapConstPtr of incoming cone map message
            */
        void coneMapCB(const utfr_msgs::msg::ConeMap &msg);

        /*! Main callback function for main_timer_;
            *  Publishes latest version of target state
            */
        void timerCB();

        /*! Get next trajectory point in trajectory if target radius is reached
            *  Target Pos and Ego Pos are compared in kWorld frame.
            */
        void getNextTrajectory();

        /*! Checks path for length, and that all three trajectory phases are present
            *
            * @returns true if path is valid
            */
        bool verifyTrajectoryPath(
            const std::vector<utfr_msgs::msg::TrajectoryPoint> &path);

        /*! Read CSV of trajectories and return a vector of trajectory points
            *
            * @param[in] string of filepath for CSV file
            * @returns vector of trajectory points
            * @throws runtime_error if file is unable to be opened
            *
            */
        std::vector<utfr_msgs::msg::TrajectoryPoint> readCSV(const std::string &filepath);

        // Publishers, Subscribers and Timers
        rclcpp::Subscription<utfr_msgs::msg::EgoState>::SharedPtr
            ego_state_subscriber_;
        rclcpp::Subscription<utfr_msgs::msg::ConeMap>::SharedPtr
            cone_map_subscriber_;
        rclcpp::Publisher<utfr_msgs::msg::TargetState>::SharedPtr
            target_state_publisher_;
        rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr
            heartbeat_publisher_;
        rclcpp::TimerBase::SharedPtr main_timer_;

        // Parameters
        double update_rate_;
        std::string mission_path_file_;
        double target_radius_;

        // Callback Variables
        utfr_msgs::msg::EgoState::SharedPtr curr_ego_state_{nullptr};
        utfr_msgs::msg::ConeMap::SharedPtr curr_cone_map_{nullptr};
        utfr_msgs::msg::TargetState::SharedPtr curr_target_{nullptr};
        utfr_msgs::msg::SystemStatus::SharedPtr curr_status_{nullptr};
        utfr_msgs::msg::TargetState::SharedPtr end_state_{nullptr};
        utfr_msgs::msg::Heartbeat heartbeat_;
        std::vector<utfr_msgs::msg::TrajectoryPoint> trajectory_path_; // Make SharedPtr?

        // Mission Variables:
        bool mission_ready_ = false;
        long trajectory_idx_ = 0;
        long trajectory_point_count_;
    };

} // namespace navigation
} // namespace utfr_dv