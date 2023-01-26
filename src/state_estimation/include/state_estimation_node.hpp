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
* file: state_estimation_node.hpp
* auth: Alyssa Wing
* desc: State estimation node header for ros2 
*       
*/

//ros2 requirements
#include <rclcpp/rclcpp.hpp> //ROS client library for c++
                            // provides ROS client functionality

//System requirements 
#include <chrono>
#include <functional>
#include <string>
#include <boost/asio.hpp>

//Transform requirements
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

//Message requirements
#include <std_msgs/msg/string.hpp>
#include <utfr_msgs/msg/ego_state.hpp>
#include <utfr_msgs/msg/cone_detections.hpp>
#include <utfr_msgs/msg/cone_map.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <utfr_msgs/msg/heartbeat.hpp>

//UTFR Comon requirements
#include <utfr_common/math.hpp>
#include <utfr_common/topics.hpp>
#include <utfr_common/frames.hpp>

//Transform (tf2) requirements
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


namespace utfr_dv {
namespace state_estimation{

class StateEstimationNode : public rclcpp::Node{
  public: 
    StateEstimationNode();
  private: 
    //Initializing parameters: 
    void initParams();  //from config.yaml

    /*! Initialize Publishers:
     * ego_state_publisher_:
     * msg: utfr_msgs::EGoState, topic: kEgoState
     * 
     * cone_map_publisher_:
     * msg: utfr_msgs::ConeDetections, topic: kConeMap
     */
    void initPublishers();

    /*! Initialize Subscribers:
     * filtered_gps_subscriber_:
     * msg: UNKNOWN/PLACEHOLDER, topic: kGPS, 
     * callback: filteredGPSCB
     * 
     * cone_detections_subscriber_:
     * msg: utfr_msgs::ConeDetections, topic: kConeDetections, 
     * callback: coneDetectionCB
     */ 
    void initSubscribers();
  
    /*! Initialize Timers:

     *  main_timer_: publishes publish_string at update_rate
          callback: timerCB() 
     */

    void initTimers();

    /*! Initialize Transform Tree.
    *  Create the pointers to the tf buffer and tf listener objects
    */
    void initTransforms();

    /*! Main callback function for main_timer_;
     *  Main processing loop where gps, imu and wheelspeed is combined to 
     *  create and publish egostate message
     */
    void timerCB();

    /*! Callback function for cone_detections_subscriber_;
     *  
     *  @param[in] msg utfr_msgs::ConeDetections Const Ptr, 
     */
    void coneDetectionsCB(const utfr_msgs::msg::ConeDetections& msg);


   /*! Send Heartbeat on every timer loop.
    *  @param[in] status current module status, using Heartbeat status enum.
    */
   
    void initHeartbeat(); 
   
    /*! Target State callback function for target_state_subscriber_
     *
     *  @param[in] msg utfr_msgs::TargetState incoming message
     */

    void publishHeartbeat(const int status); 

    /*! Publisher function for world datum
     *
     *  @param[in] datum geometry_msgs::msg::Vector3&, contains the world datum point, in LLA coordinates, as given by the GPS. Order of Lat. (Decimal Degrees), Long. (Decimal Degrees), Alt. (M)
     * 
     */
    void publishDatum(const geometry_msgs::msg::Vector3& datum); 

     /*! Callback function for gps_subsciber_
     *
     *  @param[in] datum geometry_msgs::msg::Vector3&, data from GPS
     * 
     */
    void gpsCB(const geometry_msgs::msg::Vector3& location);
  

    /*! Main Nodelet callback loop where EgoState and Cone Maps are created
     *  and published.
     */
    void nodeLoopCB();

    /*! Update base_link tow world transfrom with latest ego state
     *
     * @param[in] current_state current ego state with vehicle pose relative to 
     *            fixed world frame
     */
    void updateTransforms(
        const utfr_msgs::msg::EgoState& current_state);

    //Publishers and Timers
    rclcpp::Publisher<utfr_msgs::msg::EgoState>::SharedPtr ego_state_publisher_;
    rclcpp::Publisher<utfr_msgs::msg::ConeMap>::SharedPtr cone_map_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr 
        world_datum_publisher_;
    rclcpp::TimerBase::SharedPtr main_timer_;

    //Subscribers
    rclcpp::Subscription<utfr_msgs::msg::EgoState>::SharedPtr 
        ego_state_subscriber_;
    rclcpp::Subscription<utfr_msgs::msg::ConeMap>::SharedPtr 
        cone_map_subscriber_;
    rclcpp::Subscription<utfr_msgs::msg::ConeDetections>::SharedPtr 
        cone_detections_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr 
        gps_subscriber_;    

    //rclcpp::Subscription<utfr_msgs::msg::ConeDetections>::SharedPtr 
    //  current_detections_; //from gps cone detections
    rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr
          heartbeat_publisher_;


    //Transforms
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::TransformStamped transform_;


    //Callback Variables
    utfr_msgs::msg::ConeMap::SharedPtr current_cone_map_{nullptr};
    utfr_msgs::msg::EgoState::SharedPtr current_ego_state_{nullptr};
    utfr_msgs::msg::ConeDetections::SharedPtr current_detections_{nullptr};
    utfr_msgs::msg::Heartbeat heartbeat_;


    // Transforms
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    //Parameters
    std::string publish_string_;

    double update_rate_;
    //std::string echo_string_;
    int counter_;
    std::shared_ptr<geometry_msgs::msg::Vector3> world_datum_{nullptr};
    std::shared_ptr<geometry_msgs::msg::Vector3> curr_gps_msg_{nullptr};
};
} //namespace state_estimation
} //namespace utfr_dv
