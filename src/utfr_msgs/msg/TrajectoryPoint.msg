# Header for timestamp
std_msgs/Header header

geometry_msgs/Vector3 pos #x, y, yaw [m, m, deg]

float64 velocity #[m/s]

uint8 type # Current Vehicle Target State

# target states:
uint8 START=0
uint8 ACTIVE=1
uint8 FINISH=2