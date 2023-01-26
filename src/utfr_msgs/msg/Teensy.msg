# Header for frame
std_msgs/Header header

float64 steering_pv #[-1.0,1.0]
float64 throttle_pv #[0.0, 1.0]
float64 brake_pv #[0.0, 1.0]

# Header for wheelspeed sensor fields 
std_msgs/Header sensor_header

float64 front_left
float64 front_right
float64 rear_left
float64 rear_right

# Header for IMU data
std_msgs/Header imu_header
geometry_msgs/Vector3 imu_accel     # x, y, & z acceleration
float64 roll_rate               # about the y-axis
float64 yaw_rate                # about the z-axis

# Header for RES data
std_msgs/Header res_header
bool gonogo