#!/usr/bin/env python
import rospy
import tf
import geometry_msgs
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, TransformStamped, Point
from std_msgs.msg import Float64
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import numpy as np
import threading
from comm_node import vicon_transforms

def get_pose(x, y, z, qx = 0, qy = 0, qz = 0, qw = 1):
    ret = Pose()
    ret.position.x = x
    ret.position.y = y
    ret.position.z = z
    ret.orientation.x = qx
    ret.orientation.y = qy
    ret.orientation.z = qz
    ret.orientation.w = qw
    return ret

def dist(p1, p2):
    # both are pose.position
    return np.linalg.norm((np.array([p1.x, p1.y, p1.z]) - np.array([p2.x, p2.y, p2.z])))

def deg_wrap(ang):
    if (ang >= 0 and ang < np.pi) or (ang < 0 and ang > -np.pi):
        return ang
    elif ang < -np.pi:
        return deg_wrap(ang + 2*np.pi)
    else:
        return deg_wrap(ang - 2*np.pi)

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
        #self.obstacles_sub = rospy.Subscriber("/det/inview", Float64, callback = self.callback_obs)
        self.obstacles_sub = rospy.Subscriber("/obj/pos", Point, callback = self.callback_obs_mono)
        
        
        self.state_sub = rospy.Subscriber("/mavros/state", State, callback = self.state_cb)
        self.vicon_sub = rospy.Subscriber("/vicon/ROB498_Drone/ROB498_Drone", TransformStamped, callback = self.callback_vicon)
        self.sub_waypoints = rospy.Subscriber(node_name + '/comm/waypoints', PoseArray, self.callback_waypoints) # TODO: add back this after velocity command testing

        # # Transforms TODO: add back after velocity testing
        self.class_name_ = "CommNode::"
        self.max_vel = 0.2 #m/s
        self.goal_height = 1.6 #0.7 #m
        self.detection_dist = 1.00 #m

        # self.goal_pose = get_pose(0, 0, self.goal_height)

        self.ground_height = 0.05 #m
        self.launch = False

        self.sub_dist = 0.20 #dist between each waypoint?
        self.dist_tolerance = 0.10 #tolerance before moving to next waypoint
        self.ang_tolerance = 0.17 #0.17 #tolerance of radian before moving to next
        self.waypoints = [] # collection of pose
        self.wiggle_dist = 0 # 0.25
        self.wiggle_down = 0.25
        self.waypoints_arr = np.empty((0,3))
        self.waypoints_posearray = PoseArray()
        self.waypoints_recived = False
        self.curr_waypoint = None
        self.curr_pose = None
        self.curr_dir = np.array([1, 0])
        self.curr_quat = np.array([0, 0, 0, 1])

        # Transform stuff
        self.curr_vicon = None
        self.vicon_odom_transform = None
        self.vicon_enabled = False

        self.lock = threading.Lock()

        self.current_state = State()
        self.active = False

        self.pause = False
        self.obstacle_detected = False
        self.obs_y = None
        self.img_width = 848
        self.box_size = 1.5
        self.mono_x = 0
        self.mono_y = 0

    def direct(self, p1, p2):
        # going 1 --> 2
        # unit vector pointing to p2 from p1
        p1 = np.array([p1.x, p1.y])
        p2 = np.array([p2.x, p2.y])
        d = p2 - p1
        # self.curr_dir = d/np.linalg.norm(d)
        return d/np.linalg.norm(d)
    

    def state_cb(self, msg):
        self.current_state = msg
        if self.current_state.connected == True:
            self.active = True

    # TODO - setup vicon subscriber here and callback functions below
    def is_close(self, pose1, pose2):
        p1 = pose1.position
        p2 = pose2.position
        ang1 = tf.transformations.euler_from_quaternion((pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w))
        ang2 = tf.transformations.euler_from_quaternion((pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w))

        #print(abs(deg_wrap(ang1[2]) - deg_wrap(ang2[2])) < self.ang_tolerance)

        return dist(p1, p2) < self.dist_tolerance and abs(deg_wrap(ang1[2] - ang2[2])) < self.ang_tolerance
    
    def gen_avoidance(self):
        # generate waypoints in a box around the obstacle
        # in 1m box maybe?
        print('000000000000000 GENERATING AVOIDANCE 000000000000000')
        self.lock.acquire()

        self.push_waypoint_front(self.curr_waypoint)

        curr = self.curr_pose
        orth_unit = np.array([self.curr_dir[1], -self.curr_dir[0]])
        dir_vec_unit = np.array([self.curr_dir[0], self.curr_dir[1]])
        orth = np.array([self.curr_dir[1], -self.curr_dir[0]]) * (self.box_size/2.0)
        dir_vec = np.array([self.curr_dir[0], self.curr_dir[1]]) * self.box_size

        # start = get_pose(curr.position.x + (orth_unit[0] + self.mono_x), curr.position.y + (orth_unit[1] + self.mono_y), curr.position.z, self.curr_quat[0], self.curr_quat[1], self.curr_quat[2], self.curr_quat[3])
            
        # xy_p = np.array([curr.position.x + orth[0], curr.position.y + orth[1] + self.mono_y, curr.position.z])
        # xy_m = np.array([curr.position.x + orth[0], curr.position.y - orth[1] - self.mono_y, curr.position.z])
        point_vec = -self.mono_y * orth_unit + np.array([curr.position.x, curr.position.y])
        start = get_pose(point_vec[0], point_vec[1], curr.position.z, self.curr_quat[0], self.curr_quat[1], self.curr_quat[2], self.curr_quat[3])

        if self.mono_y < 0: # np.linalg.norm(xy_p) > np.linalg.norm(xy_m):
            # out horizontally
            print("generating in left")

            # start = get_pose(curr.position.x + (orth_unit[0]*self.mono_y) + dir_vec_unit[0] * (self.mono_x - 0.5), curr.position.y - (orth_unit[0]*self.mono_y) + dir_vec_unit[1] * (self.mono_x - 0.5), curr.position.z, self.curr_quat[0], self.curr_quat[1], self.curr_quat[2], self.curr_quat[3])

            point1 = get_pose(start.position.x + orth[0], start.position.y - orth[1], curr.position.z, self.curr_quat[0], self.curr_quat[1], self.curr_quat[2], self.curr_quat[3])
            # out and front
            point2 = get_pose(point1.position.x + dir_vec[0], point1.position.y + dir_vec[1], curr.position.z, self.curr_quat[0], self.curr_quat[1], self.curr_quat[2], self.curr_quat[3])   
            # center and front
            point3 = get_pose(point2.position.x + orth[0], point2.position.y + orth[1], curr.position.z, self.curr_quat[0], self.curr_quat[1], self.curr_quat[2], self.curr_quat[3])
                
        else:
            print("generating in right")
            # out horizontally
            # start = get_pose(curr.position.x + (orth_unit[0]*self.mono_y) + dir_vec_unit[0] * (self.mono_x - 0.5), curr.position.y + (orth_unit[0]*self.mono_y) - dir_vec_unit[1] * (self.mono_x - 0.5), curr.position.z, self.curr_quat[0], self.curr_quat[1], self.curr_quat[2], self.curr_quat[3])

            point1 = get_pose(start.position.x + orth[0], start.position.y + orth[1], curr.position.z, self.curr_quat[0], self.curr_quat[1], self.curr_quat[2], self.curr_quat[3])
            # out and front
            point2 = get_pose(point1.position.x + dir_vec[0], point1.position.y - dir_vec[1], curr.position.z, self.curr_quat[0], self.curr_quat[1], self.curr_quat[2], self.curr_quat[3])
            # center and front
            point3 = get_pose(point2.position.x + orth[0], point2.position.y - orth[1], curr.position.z, self.curr_quat[0], self.curr_quat[1], self.curr_quat[2], self.curr_quat[3])

        # reverse order bc last in first out when pushing to front
        self.push_waypoint_front(point3)
        self.push_waypoint_front(point2)
        self.push_waypoint_front(point1)
        self.push_waypoint_front(start)

        self.lock.release()
        return

    def create_turn_point(self, new_goal):
        self.lock.acquire()
        curr = self.curr_pose
        next_point = new_goal
        print('NEW GOAL: ', new_goal)

        dir_vec = self.direct(curr.position, next_point.position)
        angle = deg_wrap(np.arctan2([dir_vec[1]], [dir_vec[0]])[0])
        quat = tf.transformations.quaternion_from_euler(0, 0, angle) 
        turn_point = get_pose(curr.position.x, curr.position.y, curr.position.z, quat[0], quat[1], quat[2], quat[3])
        # turn_point = get_pose(curr.position.x, curr.position.y, self.goal_height, quat[0], quat[1], quat[2], quat[3])
        
        self.push_waypoint_front(turn_point)
        self.lock.release()
        return
    
    def create_wiggle(self, new_goal):
        ''' add new goal and also wiggle in place to try to hit waypoint 
        5cm box maybe. We can increase later
        new_goal: Pose()
        '''
        self.lock.acquire()
        print('generating wiggle points...')
        dist = self.wiggle_dist
        curr = self.curr_pose

        # safe_point = get_pose(new_goal.position.x, new_goal.position.y, self.goal_height, curr.orientation.x, curr.orientation.y, curr.orientation.z, curr.orientation.w) 

        # # self.waypoints.append(new_goal)
        # # self.waypoints.append(safe_point)

        for x in range(-1, 1, 1):
            for y in range(-1, 1, 1):
                # TODO uncomment this for real. rn test with set z position
                new = get_pose(new_goal.position.x + x*dist, new_goal.position.y + y*dist, new_goal.position.z + x*0.25, curr.orientation.x, curr.orientation.y, curr.orientation.z, curr.orientation.w)
                # new = get_pose(new_goal.position.x + x*dist, new_goal.position.y + y*dist, self.goal_height)
                self.push_waypoint_front(new)

        print("finished. \nWAYPOINTS: ", self.waypoints)
        self.lock.release()
        return

    def push_waypoint_front(self, new_goal):
        ''' add new goal to the front of the queue
        new_goal: Pose()
        '''
        #self.lock.acquire()
        print('generating new front point...')

        # safe_point = get_pose(new_goal.position.x, new_goal.position.y, self.goal_height, new_goal.orientation.x, new_goal.orientation.y, new_goal.orientation.z, new_goal.orientation.w) 

        new = [new_goal]
        # new = [safe_point]
        self.waypoints = new + self.waypoints

        print("finished. \nWAYPOINTS: ", self.waypoints)
        #self.lock.release()
        return

    def callback_waypoints(self, msg):
        if self.waypoints_recived:
            return
        print('Waypoints Received')
        self.waypoints_recived = True
        self.waypoints_posearray = msg
        for pose in msg.poses:
            pos = np.array([pose.position.x, pose.position.y, pose.position.z])
            self.waypoints_arr = np.vstack((self.waypoints_arr, pos))

        print("-------------------------")
        print("-------------------------")
        print("----WAYPOINT RECIEVED----")
        print("=========================")

    
    def waypoint_pop(self):
        next_waypoint = self.waypoints[0]
        self.waypoints = self.waypoints[1:]
        self.goal_pose = next_waypoint
        return next_waypoint

  # Callback handlers
    def handle_launch(self):
        print("+++++++++++++++++++++++++++++++++++")
        print('Launch Requested. Your drone should take off.')
        if not self.active:
            print("Cannot launch, not connected yet")
            return EmptyResponse()
        function_name = "handle_launch:"
        # only add waypoints. Main loop will do the pose publishing
        print(self.class_name_)

        self.waypoints.append(get_pose(0, 0, self.goal_height))

        # new = get_pose(3, 0.0, self.goal_height)
        # self.waypoints.append(new)

        # new = get_pose(0.0, 0.0, self.goal_height)
        # self.waypoints.append(new)

        return EmptyResponse()

    def handle_test(self):
        print('Test Requested. Your drone should perform the required tasks. Recording starts now.')
        #self.obstacle_detected = True
        #new = get_pose(0.0, 0.0, self.goal_height)
        # self.waypoints.append(new)
        # 
        # return EmptyResponse()
        if self.waypoints_arr == np.zeros((0,3)): # TODO: uncomment after velicity test
            print('have not recieved waypoint array')
            print('TEST FAIL')
            return EmptyResponse()
        else:
            for pose in self.waypoints_posearray.poses:
                # self.create_wiggle(pose)
                self.waypoints.append(pose)

        if len(self.waypoints) == self.waypoints_arr.shape[0] * 5:
            print('waypoint creation success')
        else:
            print('waypoint creation FAIL')
            print('TEST FAIL')
            return EmptyResponse()

        # check connection to realsense and can see odom messages
        if not self.active:
            print("Cannot test, not connected yet")
            return EmptyResponse()
            
        if self.curr_pose != None:
            print('can recieve odom measurements')
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
        return EmptyResponse()

    # Service callbacks
    def handle_abort(self):
        print('Abort Requested. Your drone should land immediately due to safety considerations')
        exit()
        return EmptyResponse()
    
    def callback_pose(self, pose):
        self.curr_pose = pose.pose
        q = pose.pose.orientation
        self.curr_quat = np.array([q.x, q.y, q.z, q.w])
        eul = tf.transformations.euler_from_quaternion((self.curr_quat[0], self.curr_quat[1], self.curr_quat[2], self.curr_quat[3])) 
        ang = eul[2]
        self.curr_dir = np.array([np.cos(ang), np.sin(ang)])
        return EmptyResponse()
    
    def callback_obs(self, obs):
        if obs.data == -1.0:
            self.obstacle_detected = False
        else: 
            # print("========================")
            # print("===========:)===========")
            # print("========================")
            #print(obs.data)
            self.obs_y = obs.data
            self.obstacle_detected = True
        return EmptyResponse()
    
    def callback_obs_mono(self, obs):
        if obs.z == -1.0:
            self.obstacle_detected = False
        else: 
            if (np.linalg.norm([obs.x, obs.y]) < self.detection_dist) and (self.state_machine != 2):
                # print("========================")
                # print("===========:)===========")
                # print("========================")
                #print(obs.data)
                self.mono_y = obs.y
                self.mono_x = obs.x
                self.obstacle_detected = True
            else:
                self.obstacle_detected = False
        return EmptyResponse()
    
    def callback_vicon(self, vicon):
        #print("got vicon")
        self.curr_vicon = vicon
        return EmptyResponse()

    # Our functions:
    '''
    *! Set Drone Position, publishing to waypoint target mavros topic
    *
    *  @param[in] geometry_msgs pose in FLU frame, [m/rad]
    '''
    def set_position(self, pose):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        # TODO - add frame
        curr_pose_stamped = PoseStamped()
        curr_pose_stamped.pose = self.curr_pose
        if self.vicon_enabled:
            self.vicon_odom_transform =\
                vicon_transforms.get_vicon_to_odom_transform(\
                    self.curr_vicon, curr_pose_stamped)

            pose_transformed = vicon_transforms.transform_vicon_pose(self.vicon_odom_transform, pose)
            msg.header.frame_id = 'odom'
            msg.pose = pose_transformed
            print("target pose in local: ", pose_transformed.position.x, pose_transformed.position.y, pose_transformed.position.z)
        else:
            msg.pose = pose
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

        while(not rospy.is_shutdown() and (self.current_state.mode != "OFFBOARD" and not self.current_state.armed)):
            armed = self.arming_client.call(arm_cmd)
            offboard = self.set_mode_client.call(offb_set_mode)
            print(self.current_state.mode)
            
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
        if self.vicon_enabled:
            while (self.curr_vicon is None):
                rospy.loginfo("waiting for valid current vicon")
                self.set_position(takeoff_target)
                rate.sleep()

        #self.vicon_odom_transform =\
        #    vicon_transforms.get_vicon_to_odom_transform(\
        #        self.curr_vicon, self.curr_pose)

        print(self.curr_pose)
        self.curr_waypoint = self.waypoint_pop()
        self.state_machine = 2
        self.state_machine_counter = 0
        self.obs_avoid_counter = 0
        while(not rospy.is_shutdown()):
            if self.current_state.mode != "OFFBOARD":
                offboard = self.set_mode_client.call(offb_set_mode)
                print(self.current_state.mode)
                
                if(offboard.mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")

            if self.vicon_enabled:
                pose_to_compare = Pose()
                pose_to_compare.position.x = self.curr_vicon.transform.translation.x
                pose_to_compare.position.y = self.curr_vicon.transform.translation.y
                pose_to_compare.position.z = self.curr_vicon.transform.translation.z
                
            else:
                pose_to_compare = self.curr_pose

            # if obstacle detected, do obstaclle avoidance and self.pause is true. Only after obstacle avoidance is done, self.pause false and continue on way
            if self.obstacle_detected:
                if self.pause == False:
                    print("+++++++++++++++++GENERATING")
                    self.pause = True
                    # do generation
                    self.gen_avoidance()
                    self.curr_waypoint = self.waypoint_pop()
                    self.obs_avoid_counter = 0

                
            if self.curr_waypoint is None or self.is_close(self.curr_waypoint, pose_to_compare): #last check should be redundant but better safe than sorry
                if len(self.waypoints) > 0:
                    # print("current waypoint reached at : ", self.curr_waypoint.position.x, self.curr_waypoint.position.y, self.curr_waypoint.position.z)
                    
                    if self.pause == False:
                        # if isturn = FALSE gen turn push to self.waypoints and set isturn = TRUE
                        # else: isturn = FALSE 
                        if (self.state_machine == 1): #final wigglepoint hit
                            print("===============WIGGLEPOINTs FINISHED=============")
                            self.create_turn_point(self.waypoints[0])
                            self.state_machine = 2

                            self.curr_waypoint = self.waypoint_pop()

                        elif (self.state_machine == 2): #turnpoint finished
                            print("===============TURNPOINT HIT=============")
                            curr_quat = self.curr_waypoint.orientation
                            self.curr_waypoint = self.waypoint_pop()
                            self.curr_waypoint.orientation = curr_quat
                            self.state_machine = 3
                            
                        elif (self.state_machine == 3): #waypoint hit / in the middle of wigglepoints
                            print("===============IN 3 RIGHT NOW=============")
                            print("CURRENT WAYPOINTS", self.waypoints)
                            # self.obstacle_detected = True
                            # continue
                            if self.state_machine_counter == 0: # waypoint hit : create wigglepoints
                                print("===============WAYPOINT HIT=============")
                                self.create_wiggle(self.curr_waypoint)
                                self.state_machine_counter += 1
                            elif self.state_machine_counter == 3: # final wigglepoint hit
                                print("===============3/4 WIGGLE DONE=============")
                                self.state_machine = 1
                                self.state_machine_counter = 0
                            else:
                                print("===============" + str(self.state_machine_counter) + "/4 WIGGLE DONE=============")
                                self.state_machine_counter += 1
                                
                            self.curr_waypoint = self.waypoint_pop()

                        print("new waypoint : ", self.curr_waypoint.position.x, self.curr_waypoint.position.y, self.curr_waypoint.position.z)
                        

            if self.pause == True:
                print("===============PAUSED=============")
                pose_to_compare = self.curr_pose
                # check that the avoidance waypoints have been reached
                print("===============%i/3 AVOID DONE=============" %self.obs_avoid_counter)
                if self.obs_avoid_counter < 3:
                    if self.curr_waypoint is None or self.is_close(self.curr_waypoint, pose_to_compare): #last check should be redundant but better safe than sorry
                        if len(self.waypoints) > 0:
                            print("===============%i/3 AVOID DONE=============" %self.obs_avoid_counter)
                            # pop 3 obstacle avoidance waypoints
                            self.curr_waypoint = self.waypoint_pop()
                            self.obs_avoid_counter += 1
                            print("new waypoint : ", self.curr_waypoint.position.x, self.curr_waypoint.position.y, self.curr_waypoint.position.z)
                else:
                    # reset variables and continue on path
                    print("===============AVOIDANCE DONE=============")
                    self.obs_avoid_counter = 0
                    self.pause = False
                    self.obstacle_detected = False

                           
            self.set_position(self.curr_waypoint)
            print("CURR pose in local: ", round(self.curr_pose.position.x, 3), round(self.curr_pose.position.y, 3), round(self.curr_pose.position.z, 3))
            eul = np.round(tf.transformations.euler_from_quaternion((self.curr_pose.orientation.x, self.curr_pose.orientation.y, self.curr_pose.orientation.z, self.curr_pose.orientation.w)), 3)
            print("CURR orientation in local: ", eul[0], eul[1], eul[2])
            #print("CURR DIR ", self.curr_dir)
            if self.vicon_enabled:
                print("CURR pose in vicon: ", self.curr_vicon.transform.translation.x, self.curr_vicon.transform.translation.y, self.curr_vicon.transform.translation.z)
            print(" ")
            curr_pose_stamped = PoseStamped()
            curr_pose_stamped.pose = self.curr_pose
            if self.vicon_enabled:
                print("TARGET pose in vicon: ", self.curr_waypoint.position.x, self.curr_waypoint.position.y, self.curr_waypoint.position.z)
            else:
                print("TARGET pose in local: ", round(self.curr_waypoint.position.x, 3), round(self.curr_waypoint.position.y, 3), round(self.curr_waypoint.position.z, 3))
                eul = np.round(tf.transformations.euler_from_quaternion((self.curr_waypoint.orientation.x, self.curr_waypoint.orientation.y, self.curr_waypoint.orientation.z, self.curr_waypoint.orientation.w)), 3)
                print("TARGET orientation in local: ", eul[0], eul[1], eul[2])
            
            print(" ")
            print(" ")

            rate.sleep()

        
if __name__ == "__main__":
    # print("im here")
    # xxx
    # exit()
    comm_node = CommNode()
    comm_node.run()
    rospy.spin()
    # rosservice call rob498_drone_13/comm/launch
    # rosservice call rob498_drone_13/comm/land
