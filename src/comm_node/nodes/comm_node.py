#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, PoseArray
from tf import TransformBroadcaster, TransformListener
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import numpy as np

class CommNode:
    def __init__(self):
        print('This is a dummy drone node to test communication with the ground control')
        print('node_name should be rob498_drone_TeamID. Service topics should follow the name convention below')
        print('The TAs will test these service calls prior to flight')
        print('Your own code should be integrated into this node')

        node_name = 'rob498_drone_13'
        rospy.init_node(node_name) 
        self.srv_launch = rospy.Service(node_name + '/comm/launch', Empty, self.callback_launch)
        self.srv_test = rospy.Service(node_name + '/comm/test', Empty, self.callback_test)
        self.srv_land = rospy.Service(node_name + '/comm/land', Empty, self.callback_land)
        self.srv_abort = rospy.Service(node_name + '/comm/abort', Empty, self.callback_abort)

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)  


        # Pubs, Subs and Transforms
        self.setpoint_pub_ = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback = self.pose_cb) # in map frame
        self.vicon_sub = rospy.Subscriber("/vicon/ROB498_Drone/ROB498_Drone", TransformStamped, callback = self.vicon_cb)
        self.state_sub = rospy.Subscriber("/mavros/state", State, callback = self.state_cb)
        self.sub_waypoints = rospy.Subscriber(node_name+'/comm/waypoints', PoseArray, self.callback_waypoints)

        # Transforms
        self.transform_broadcaster = TransformBroadcaster()
        self.transform_listener = TransformListener()
        self.vicon_frame = "base_link" # TODO change this back "vicon"
        self.baselink_frame = "base_link"
        self.odom_frame = "odom" 

        self.class_name_ = "CommNode::"
        self.max_vel = 0.2 #m/s
        self.goal_height = 1.6 #m

        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = self.vicon_frame
        self.goal_pose.pose.position.x = 0
        self.goal_pose.pose.position.y = 0
        self.goal_pose.pose.position.z = self.goal_height

        self.ground_height = 0.1 #m
        self.launch = False

        self.sub_dist = 0.1 #dist between each waypoint?
        self.dist_tolerance = 0.1 #tolerance before moving to next waypoint
        self.waypoints = []
        self.waypoints_arr = np.empty((0,3))
        self.curr_waypoint = None
        self.curr_pose = None
        self.waypoints_recived = False

        self.current_state = State()
        self.active = False

    # TODO - setup vicon subscriber here and callback functions below
    def is_close(self, pose1, pose2):
        p1 = pose1.pose.position
        p2 = pose2.pose.position
        dist = np.linalg.norm((np.array([p1.x, p1.y, p1.z]) - np.array([p2.x, p2.y, p2.z])))
        return dist < self.dist_tolerance

    # SAME CODE AS OTHER BUT WRONG FRAME i THINK
    # def create_waypoints(self):
    #     print('generating waypoints...')
    #     gen_points = True

    #     # right now, curr_pose should just be on the ground the only change from initial pose to goal is z
    #     self.goal_pose = Pose()
    #     cp = self.curr_pose.position # so it doesnt change half way lol
    #     self.goal_pose.position.x = cp.x
    #     self.goal_pose.position.y = cp.y
    #     self.goal_pose.position.z = self.goal_height

    #     while gen_points:
    #         if self.waypoints[-1].position.z == self.goal_height:
    #             gen_points = False
    #             break
    #         if self.is_close(self.goal_pose, self.waypoints[-1]):
    #             self.waypoints.append(goal_pose_stamp)
    #             gen_points = False
    #             break

    #         point = Pose()
    #         point.header.frame_id = head
    #         point.position.x = cp.x
    #         point.position.y = cp.y
    #         point.position.z = cp.z + self.sub_dist

    #         self.waypoints.append(point)

    #     print("finished. \nWAYPOINTS:")
    #     print(self.waypoints)

    def callback_waypoints(self, msg):
        if self.waypoints_recived:
            return
        print('Waypoints Received')
        self.waypoints_recived = True
        for pose in msg.poses:
            pos = np.array([pose.position.x, pose.position.y, pose.position.z])
            self.waypoints_arr = np.vstack((self.waypoints_arr, pos))

    def vicon_cb(self, msg):
        # Creates TF connection from child_frame (self.vicon_frame?) to header_frame (self.baselink_frame?)
        self.transform_broadcaster(msg.transform.translation, 
                                   msg.transform.rotation, 
                                   rospy.Time.now(),
                                   msg.child_frame_id,
                                   msg.header.frame_id)

    def state_cb(self, msg):
        self.current_state = msg
        if self.current_state.connected == True:
            self.active = True

    def is_close(self, pose1, pose2):
        p1 = pose1.position
        p2 = pose2.position
        dist = np.linalg.norm((np.array([p1.x, p1.y, p1.z]) - np.array([p2.x, p2.y, p2.z])))
        return dist < self.dist_tolerance

    def create_waypoints(self):
        print('generating waypoints...')
        gen_points = True

        # right now, we should be in the air. hovering at z = 1.5
        for i in range(self.waypoints_arr.shape[0]):
            # generate pose stamped messages to send to the drone

            point = PoseStamped()
            point.header.frame_id = self.vicon_frame # rn vicon frame kinda broken. Might have to switch back to drone frame
            point.pose.position.x = self.waypoints_arr[i, 0]
            point.pose.position.y = self.waypoints_arr[i, 1]
            point.pose.position.z = self.waypoints_arr[i, 2]
            point.pose.orientation.x = 0
            point.pose.orientation.y = 0
            point.pose.orientation.z = 0
            point.pose.orientation.w = 1

            # TODO NEED TO TRANSFORM IT BACK TO DRONE FRAME TO SEND OUT POSITION THINGS

            self.waypoints.append(point)

        print("finished. \nWAYPOINTS:")
        print(self.waypoints)
    
    def waypoint_pop(self):
        next_waypoint = self.waypoints[0]
        self.waypoints = self.waypoints[1:]
        self.goal_pose = next_waypoint
        return next_waypoint

  # Callback handlers
    def handle_launch(self):
        # handler for task 2
        print('Launch Requested. Your drone should take off.')
        if not self.active:
            print("Cannot launch, not connected yet")
            return EmptyResponse()
        
        function_name = "handle_launch:"
        # only add waypoints. Main loop will do the pose publishing
        print(self.class_name_)

        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = self.vicon_frame
        self.goal_pose.pose.position.x = 0.0
        self.goal_pose.pose.position.y = 0.0
        self.goal_pose.pose.position.z = self.goal_height
        
        self.goal_pose.pose.orientation.x = 0
        self.goal_pose.pose.orientation.y = 0
        self.goal_pose.pose.orientation.z = 0
        self.goal_pose.pose.orientation.w = 1
        print("goal: ", self.goal_pose)
        
        self.waypoints.append(self.goal_pose)
        self.curr_waypoint = self.waypoint_pop()

    def handle_test(self):
        print('Test Requested. Your drone should perform the required tasks. Recording starts now.')
        
        ######## okay lol uncomment for #3
        if self.waypoints_arr == np.zeros((0,3)):
            print('have not recieved waypoint array')
            print('TEST FAIL')
            return EmptyResponse()
        else:
            self.create_waypoints()
        if len(self.waypoints) == self.waypoints_arr.shape[0]:
            print('waypoint creation success')
        else:
            print('waypoint creation FAIL')
            print('TEST FAIL')
            return EmptyResponse()

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

        takeoff_target = PoseStamped()
        takeoff_target.header.frame_id = self.vicon_frame
        takeoff_target.pose.position.x = 0.0
        takeoff_target.pose.position.y = 0.0
        takeoff_target.pose.position.z = self.goal_height/4
        takeoff_target.pose.orientation.x = 0
        takeoff_target.pose.orientation.y = 0
        takeoff_target.pose.orientation.z = 0
        takeoff_target.pose.orientation.w = 1
        self.waypoints.append(takeoff_target)
        print("first goal: ", takeoff_target)

        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = self.vicon_frame
        self.goal_pose.pose.position.x = 0.0
        self.goal_pose.pose.position.y = 0.0
        self.goal_pose.pose.orientation.x = 0
        self.goal_pose.pose.orientation.y = 0
        self.goal_pose.pose.orientation.z = 0
        self.goal_pose.pose.orientation.w = 1
        print("goal: ", self.goal_pose)
        self.waypoints.append(self.goal_pose)
        
        self.curr_waypoint = self.waypoint_pop() 


    def handle_abort(self):
        print('Abort Requested. Your drone should land immediately due to safety considerations')
        exit()
        return EmptyResponse()

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
    
    def pose_cb(self, pose):
        self.curr_pose = pose.pose
        return EmptyResponse()

    # Our functions:
    '''
    *! Set Drone Position, publishing to waypoint target mavros topic
    *
    *  @param[in] geometry_msgs pose in FLU frame, [m/rad]
    '''
    def set_position(self, poseStamped):
        # Transform incoming from vicon to baselink at the latest time
        # msg = self.transform_listener.transformPose(self.odom_frame, poseStamped)

        poseStamped.header.stamp = rospy.Time.now()
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

        takeoff_target = PoseStamped()
        takeoff_target.header.frame_id = self.vicon_frame
        takeoff_target.pose.position.x = 0.0
        takeoff_target.pose.position.y = 0.0
        takeoff_target.pose.position.z = 0.0       
        takeoff_target.pose.orientation.x = 0
        takeoff_target.pose.orientation.y = 0
        takeoff_target.pose.orientation.z = 0
        takeoff_target.pose.orientation.w = 1

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

