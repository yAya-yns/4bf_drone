#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import numpy as np
import threading

def get_pose(x, y, z):
    ret = Pose()
    ret.position.x = x
    ret.position.y = y
    ret.position.z = z
    ret.orientation.x = 0
    ret.orientation.y = 0
    ret.orientation.z = 0
    ret.orientation.w = 1
    return ret

def dist(p1, p2):
    # both are pose.position
    return np.linalg.norm((np.array([p1.x, p1.y, p1.z]) - np.array([p2.x, p2.y, p2.z])))

def direct(p1, p2):
    # going 1 --> 2
    p1 = np.array([p1.x, p1.y, p1.z])
    p2 = np.array([p2.x, p2.y, p2.z])
    d = p2 - p1
    return d/np.linalg.norm


class CommNode:
    def __init__(self):
        print('This is a dummy drone node to test communication with the ground control')
        print('node_name should be rob498_drone_TeamID. Service topics should follow the name convention below')
        print('The TAs will test these service calls prior to flight')
        print('Your own code should be integrated into this node')
        
        node_name = 'rob498_drone_13'
        rospy.init_node(node_name) 
        self.srv_launch = rospy.Service(node_name + '/comm/launch', Empty, lambda r: self.handle_launch())
        self.srv_test = rospy.Service(node_name + '/comm/test', Empty, lambda r: self.handle_test())
        self.srv_land = rospy.Service(node_name + '/comm/land', Empty, lambda r: self.handle_land())
        self.srv_abort = rospy.Service(node_name + '/comm/abort', Empty, lambda r: self.handle_abort())

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)  

        # Pubs, Subs and Transforms
        self.setpoint_pub_ = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback = self.callback_pose)
        
        self.state_sub = rospy.Subscriber("/mavros/state", State, callback = self.state_cb)
        # self.sub_waypoints = rospy.Subscriber(node_name+'/comm/waypoints', PoseArray, self.callback_waypoints) TODO: add back this after velocity command testing

        # # Transforms TODO: add back after velocity testing
        # self.transform_broadcaster = TransformBroadcaster()
        # self.transform_listener = TransformListener()
        # self.vicon_frame = "base_link" # TODO change this back "vicon"
        # self.baselink_frame = "base_link"
        # self.odom_frame = "odom" 
        self.class_name_ = "CommNode::"
        self.max_vel = 0.2 #m/s
        self.goal_height = 1.6 #m

        self.goal_pose = get_pose(0, 0, self.goal_height)

        self.ground_height = 0.1 #m
        self.launch = False

        self.sub_dist = 0.1 #dist between each waypoint?
        self.dist_tolerance = 0.1 #tolerance before moving to next waypoint
        self.waypoints = [] # collection of pose
        self.waypoints_arr = np.empty((0,3))
        self.waypoints_recived = False
        self.curr_waypoint = None
        self.curr_pose = None
        self.lock = threading.Lock()

        self.current_state = State()
        self.active = False

    def state_cb(self, msg):
        self.current_state = msg
        if self.current_state.connected == True:
            self.active = True

    # TODO - setup vicon subscriber here and callback functions below
    def is_close(self, pose1, pose2):
        p1 = pose1.position
        p2 = pose2.position
        dist = dist(p1, p2)
        return dist < self.dist_tolerance

    def create_waypoints(self, new_goal):
        '''TODO: for challenge 3
        generate new set of waypoints for each new goal point (This way can be used for challenge 2 and 3)'''
        
        print('generating waypoints...')
        gen_points = True
        self.lock.acquire()
        last_waypoint = self.waypoints[-1]
        direction = direct(last_waypoint.position, new_goal.position) # unit vector from current to new goal
        
        # keep adding sub waypoints from the last added point until we read the new goal
        while gen_points:
            # check how close we are
            if self.is_close(new_goal, self.waypoints[-1]):
                self.waypoints.append(new_goal)
                gen_points = False
                break
            dist = dist(new_goal.position, self.waypoints[-1].position)
            p1 = np.array([self.waypoints[-1].position.x, self.waypoints[-1].position.y, self.waypoints[-1].position.z])
            p2 = np.array([new_goal.position.x, new_goal.position.y, new_goal.position.z])
            # use sub distance
            new = p1 + direction * self.sub_dist

            point = get_pose(new[0], new[1], new[2])

            self.waypoints.append(point)

        print("finished. \nWAYPOINTS:")
        print(self.waypoints)
        self.lock.release()

    # def callback_waypoints(self, msg):
    #     if self.waypoints_recived:
    #         return
    #     print('Waypoints Received')
    #     self.waypoints_recived = True
    #     for pose in msg.poses:
    #         pos = np.array([pose.position.x, pose.position.y, pose.position.z])
    #         self.waypoints_arr = np.vstack((self.waypoints_arr, pos))
    
    def waypoint_pop(self):
        next_waypoint = self.waypoints[0]
        self.waypoints = self.waypoints[1:]
        self.goal_pose = next_waypoint
        return next_waypoint

  # Callback handlers
    def handle_launch(self):
        print('Launch Requested. Your drone should take off.')
        if not self.active:
            print("Cannot launch, not connected yet")
            return EmptyResponse()
        function_name = "handle_launch:"
        # only add waypoints. Main loop will do the pose publishing
        print(self.class_name_)

        self.goal_pose = get_pose(0.0, 0.0, self.goal_height)
        self.create_waypoints(self.goal_pose)


        #   COMMENTED OUT TO TEST SUBWAY POINT GENERATION 
        # self.goal_pose = get_pose(0.0, 0.0, self.goal_height)
        # print("goal: ", self.goal_pose)
        
        # self.waypoints.append(self.goal_pose)
        # self.curr_waypoint = self.waypoint_pop()

    def handle_test(self):
        print('Test Requested. Your drone should perform the required tasks. Recording starts now.')
        
        ######### okay lol uncomment for #3
        # if self.waypoints_arr == np.zeros((0,3)): # TODO: uncomment after velicity test
        #     print('have not recieved waypoint array')
        #     print('TEST FAIL')
        #     return EmptyResponse()
        # else:
        #     self.create_waypoints()
        # if len(self.waypoints) == self.waypoints_arr.shape[0]:
        #     print('waypoint creation success')
        # else:
        #     print('waypoint creation FAIL')
        #     print('TEST FAIL')
        #     return EmptyResponse()

        # check if on ground, check connection to realsense and can see odom messages
        if not self.active:
            print("Cannot test, not connected yet")
            return EmptyResponse()
            
        if self.curr_pose != None:
            print('can recieve odom measurements')
            if self.curr_pose.position.z < self.ground_height:
                print('on ground and can recieve pose measurements')
                print('TEST SUCCESS')
                return EmptyResponse()
            else:
                print('not on the ground. Cannot start')
                print('TEST FAIL')
                return EmptyResponse()
        else:
            print('CANNOT recieve odom measurements')
            print('TEST FAIL')
            return EmptyResponse()
      

    def handle_land(self):
        print('Land Requested. Your drone should land.')
        function_name = "handle_land:"
        if not self.active:
            print("Cannot launch, not connected yet")
            return EmptyResponse()
        print(self.class_name_)

        # clear waypoints and land immediately
        # only add waypoints. Main loop will do the pose publishing
        self.waypoints = []

        takeoff_target = get_pose(0.0, 0.0, self.goal_height/4)
        self.waypoints.append(takeoff_target)
        print("first goal: ", takeoff_target)

        self.goal_pose = get_pose(0.0, 0.0, 0.0)
        print("goal: ", self.goal_pose)
        self.waypoints.append(self.goal_pose)
        
        self.curr_waypoint = self.waypoint_pop() 

    # Service callbacks
    def handle_abort(self):
        print('Abort Requested. Your drone should land immediately due to safety considerations')
        exit()
        return EmptyResponse()
    
    def callback_pose(self, pose):
        self.curr_pose = pose.pose
        return EmptyResponse()

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
        msg.header.frame_id = 'map'
        self.setpoint_pub_.publish(msg)
    

    def run(self):
        rate = rospy.Rate(20)

        print("Connecting...")
        while((not rospy.is_shutdown() and not self.current_state.connected) or (self.curr_pose is None)):
            rate.sleep()
        print("Connected!")
        self.active = True

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        takeoff_target = get_pose(0.0, 0.0, 0.0)

        while(not rospy.is_shutdown() and self.current_state.mode != "OFFBOARD" and not self.current_state.armed):
            armed = self.arming_client.call(arm_cmd)
            offboard = self.set_mode_client.call(offb_set_mode)

            if(offboard.mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            elif(armed.success == True):
                rospy.loginfo("Vehicle armed")
                
            self.set_position(takeoff_target)
            rate.sleep()

        self.waypoints.append(takeoff_target)

        while (self.curr_pose is None):
            rospy.loginfo("waiting for valid current pose")
            self.set_position(takeoff_target)
            rate.sleep()

        print(self.curr_pose)
        self.curr_waypoint = self.waypoint_pop()
        while(not rospy.is_shutdown()):
            if self.curr_waypoint is None or self.is_close(self.curr_waypoint, self.curr_pose) or self.is_close(self.goal_pose, self.curr_pose): #last check should be redundant but better safe than sorry
                if len(self.waypoints) > 0:
                    # print("current waypoint reached at : ", self.curr_waypoint.position.x, self.curr_waypoint.position.y, self.curr_waypoint.position.z)
                    self.curr_waypoint = self.waypoint_pop()
                    print("new waypoint : ", self.curr_waypoint.position.x, self.curr_waypoint.position.y, self.curr_waypoint.position.z)
                    
            self.set_position(self.curr_waypoint)
            print("curr pose: ", self.curr_pose.position.x, self.curr_pose.position.y, self.curr_pose.position.z)
            print("target pose: ", self.curr_waypoint.position.x, self.curr_waypoint.position.y, self.curr_waypoint.position.z)
            rate.sleep()

        
if __name__ == "__main__":
    comm_node = CommNode()
    comm_node.run()
    rospy.spin()
    # rosservice call rob498_drone_13/comm/launch
    # rosservice call rob498_drone_13/comm/land