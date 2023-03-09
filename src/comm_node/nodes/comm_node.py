#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler

class CommNode:
  def __init__(self):
    print('This is a dummy drone node to test communication with the ground control')
    print('node_name should be rob498_drone_TeamID. Service topics should follow the name convention below')
    print('The TAs will test these service calls prior to flight')
    print('Your own code should be integrated into this node')
    
    node_name = 'rob498_drone_13'
    rospy.init_node(node_name) 
    srv_launch = rospy.Service(node_name + '/comm/launch', Empty, self.callback_launch)
    srv_test = rospy.Service(node_name + '/comm/test', Empty, self.callback_test)
    srv_land = rospy.Service(node_name + '/comm/land', Empty, self.callback_land)
    srv_abort = rospy.Service(node_name + '/comm/abort', Empty, self.callback_abort)

    # Your code goes below
    setpoint_topic = "/mavros/setpoint_position/local"
    self.setpoint_pub_ = rospy.Publisher(setpoint_topic, PoseStamped, queue_size=10)
    self.class_name_ = "CommNode::"

    # TODO - setup vicon subscriber here and callback functions below

  # Callback handlers
  def handle_launch(self):
      #print('Launch Requested. Your drone should take off.')

      function_name = "handle_launch:"
      # Two tier'd takeoff - takeoff to 0.5m first, delay, then takeoff to full 14m
      print(self.class_name_ + function_name + "taking off to 0.5m")
      takeoff_target = Pose()
      takeoff_target.position.x = 0.0
      takeoff_target.position.y = 0.0
      takeoff_target.position.z = 0.5
      takeoff_target.orientation = quaternion_from_euler(0.0, 0.0, 0.0) 

      self.set_position(takeoff_target)
      print(self.class_name_ + function_name + "waiting for 5s...")

      rospy.sleep(5.)
      print(self.class_name_ + function_name + "waiting for 14m")
      takeoff_target = Pose()
      takeoff_target.position.x = 0.0
      takeoff_target.position.y = 0.0
      takeoff_target.position.z = 14.0
      takeoff_target.orientation = quaternion_from_euler(0.0, 0.0, 0.0) 
      self.set_position(takeoff_target)
      print(self.class_name_ + function_name + "takeoff complete")

  def handle_test(self):
      print('Test Requested. Your drone should perform the required tasks. Recording starts now.')

  def handle_land(self):
      print('Land Requested. Your drone should land.')

  def handle_abort(self):
      print('Abort Requested. Your drone should land immediately due to safety considerations')

  # Service callbacks
  def callback_launch(self,request):
      self.handle_launch()
      return EmptyResponse()

  def callback_test(self,request):
      self.handle_test()
      return EmptyResponse()

  def callback_land(self,request):
      self.handle_land()
      return EmptyResponse()

  def callback_abort(self,request):
      self.handle_abort()
      return EmptyResponse()

  # Our functions:

  # Set Drone Position, publishing to waypoint target mavros topic
  #
  # @param[in] geometry_msgs pose in FLU frame, [m/rad]
  def set_position(self, pose):
      msg = PoseStamped()
      msg.pose = pose
      msg.header.stamp = rospy.Time.now()
      # TODO - add frame
      # msg.header.frame_id = "base_link"
      self.setpoint_pub_.publish(msg)
    
if __name__ == "__main__":
    comm_node = CommNode()
    rospy.spin()