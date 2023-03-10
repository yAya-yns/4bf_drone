#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler
import numpy as np

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
        self.local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = self.callback_pose)
        self.class_name_ = "CommNode::"
        self.max_vel = 0.2 #m/s
        self.goal_height = 0.5 #m
        self.goal_pose = None
        self.ground_height = 0.2 #m

        self.sub_dist = 0.1 #dist between each waypoint?
        self.dist_tolerance = 0.1 #tolerance before moving to next waypoint
        self.waypoints = []
        self.curr_waypoint = Pose()
        self.curr_pose = None

    # TODO - setup vicon subscriber here and callback functions below
    def is_close(self, pose1, pose2):
        p1 = pose1.position
        p2 = pose2.position
        dist = np.linalg.norm((np.array([p1.x, p1.y, p1.z]) - np.array([p2.x, p2.y, p2.z])))
        return dist < self.dist_tolerance

    def create_waypoints(self):
        print('generating waypoints...')
        gen_points = True

        # right now, curr_pose should just be on the ground the only change from initial pose to goal is z
        self.goal_pose = Pose()
        cp = self.curr_pose.position # so it doesnt change half way lol
        self.goal_pose.position.x = cp.x
        self.goal_pose.position.y = cp.y
        self.goal_pose.position.z = self.goal_height

        while gen_points:
            if self.waypoints[-1].position.z == self.goal_height:
                gen_points = False
                break
            if self.is_close(self.goal_pose, self.waypoints[-1]):
                self.waypoints.append(goal_pose_stamp)
                gen_points = False
                break

            point = Pose()
            point.header.frame_id = head
            point.position.x = cp.x
            point.position.y = cp.y
            point.position.z = cp.z + self.sub_dist

            self.waypoints.append(point)

        print("finished. \nWAYPOINTS:")
        print(self.waypoints)

  # Callback handlers
    def handle_launch(self):
        print('Launch Requested. Your drone should take off.')
        # for w_point in waypoint:
        #     w_pose = w_point.position
        #     print(self.class_name_ + function_name + " sending to ", w_pose.x, w_pose.y, w_pose.z)
        #     self.set_position(w_pose)
        function_name = "handle_launch:"
        # Two tier'd takeoff - takeoff to 0.5m first, delay, then takeoff to full 14m
        print(self.class_name_)
        takeoff_target = Pose()
        takeoff_target.position.x = 0.0
        takeoff_target.position.y = 0.0
        takeoff_target.position.z = self.goal_height/2
        
        takeoff_target.orientation.x = 0
        takeoff_target.orientation.y = 0
        takeoff_target.orientation.z = 0
        takeoff_target.orientation.w = 1
        print(takeoff_target)
        self.set_position(takeoff_target)
        while (not self.is_close(self.curr_pose, takeoff_target)):
            print(self.class_name_ + function_name + "waiting for initial takeoff. Currently at:")
            print(self.curr_pose.position)

        print(self.class_name_ + function_name + "waiting for 5s...")

        rospy.sleep(5.)
        print(self.class_name_ + function_name + "waiting for 0.5m")
        takeoff_target = Pose()
        takeoff_target.position.x = 0.0
        takeoff_target.position.y = 0.0
        takeoff_target.position.z = self.goal_height
        takeoff_target.orientation.x = 0
        takeoff_target.orientation.y = 0
        takeoff_target.orientation.z = 0
        takeoff_target.orientation.w = 1
        self.set_position(takeoff_target)
        while (not self.is_close(self.curr_pose, takeoff_target)):
            print(self.class_name_ + function_name + "waiting for final takeoff. Currently at:")
            print(self.curr_pose.position)
        print(self.class_name_ + function_name + "takeoff complete")

    def handle_test(self):
        print('Test Requested. Your drone should perform the required tasks. Recording starts now.')
        
        ######### okay lol uncomment for #3
        #   self.create_waypoints()
        #   if len(self.waypoints)!= 0:
        #     print('waypoint creation success')
        #   else:
        #     print('waypoint creation FAIL')
        #     print('TEST FAIL')
        #     return 0

        # check if on ground, check connection to realsense and can see odom messages
        if self.curr_pose != None:
            print('can recieve odom measurements')
            if self.curr_pose.position.z < self.ground_height:
                print('on ground and can recieve pose measurements')
                print('TEST SUCCESS')
                return 1
            else:
                print('not on the ground. Cannot start')
                print('TEST FAIL')
                return 0
        else:
            print('CANNOT recieve odom measurements')
            print('TEST FAIL')
            return 0
      

    def handle_land(self):
        print('Land Requested. Your drone should land.')
        function_name = "handle_land:"
        # Two tier'd takeoff - takeoff to 0.5m first, delay, then takeoff to full 14m
        print(self.class_name_)
        takeoff_target = Pose()
        takeoff_target.position.x = 0.0
        takeoff_target.position.y = 0.0
        takeoff_target.position.z = self.goal_height/4
        takeoff_target.orientation = quaternion_from_euler(0.0, 0.0, 0.0) 

        self.set_position(takeoff_target)
        print(self.class_name_ + function_name + "waiting for 5s...")

        rospy.sleep(5.)
        print(self.class_name_ + function_name + "waiting for hover above ground")
        takeoff_target = Pose()
        takeoff_target.position.x = 0.0
        takeoff_target.position.y = 0.0
        takeoff_target.position.z = self.ground_height
        takeoff_target.orientation = quaternion_from_euler(0.0, 0.0, 0.0) 
        self.set_position(takeoff_target)
        print(self.class_name_ + function_name + "hover complete")
        rospy.sleep(5.)
        exit()


    def handle_abort(self):
        print('Abort Requested. Your drone should land immediately due to safety considerations')
        exit()

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
    
    def callback_pose(self, pose):
        self.curr_pose = pose.pose
        return 0

    # Our functions:
    '''
    *! Set Drone Position, publishing to waypoint target mavros topic
    *
    *  @param[in] geometry_msgs pose in FLU frame, [m/rad]
    '''
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
