'''
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
* file: perception_node.py
* auth: Kelvin Cui
* desc: perception node main file
'''
# System Requirements
import cv2
import numpy as np

# ROS2 Requirements
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError

# Message Requirements
from sensor_msgs.msg import Image
from utfr_msgs.msg import ConeDetections
from utfr_msgs.msg import Heartbeat

# Service Requirements
from std_srvs.srv import Trigger

# Service Requirements
from std_srvs.srv import Trigger

class PerceptionNode(Node):
  def __init__(self):
    super().__init__("perception_node")

    self.loadParams()
    self.initSubscribers()
    self.initPublishers()
    self.initServices()
    self.initTimers()

    #Initialize callback variables:
    self.left_img_recieved_ = False
    self.right_img_recieved_ = False
    self.first_img_arrived_ = False


    #Initialize image conversion bridge:
    self.bridge = CvBridge()

    #Initialize heartbeat
    self.initHeartbeat()

  def loadParams(self):
    """ 
    Initialize and load params from config.yaml:

    baseline : double:
      distance between camera centers

    left_topic : string:
      name of left image publisher topic

    cone_detections_topic : string:
      name of cone detections publisher topic

    heartbeat_topic : string:
      name of heartbeat publisher topic

    update_rate : double:
      rate of capture, processing and publishing in [ms]

    distortion : array:
      distortion coefficients for undistorting frames from camera

    instrinsics : array:
      intrinsic camera matrix for 3d coordinate mapping, in a single 9x1 array
    
    """
    self.declare_parameter(\
        'baseline', 10.0)
    self.declare_parameter(\
        'left_camera_topic', '/left_camera_node/images')
    self.declare_parameter(\
        'right_camera_topic', '/right_camera_node/images')
    self.declare_parameter(\
        'cone_detections_topic', "perception/cone_detections")
    self.declare_parameter(\
        'heartbeat_topic', "/perception/heartbeat")
    self.declare_parameter(\
        'update_rate', 33.33)
    self.declare_parameter(\
        'distortion', [0.0])
    self.declare_parameter(\
        'intrinsics', [0.0])

    self.baseline_ = self.get_parameter(
        'baseline').get_parameter_value().double_value
    self.left_camera_topic = self.get_parameter(
        'left_camera_topic').get_parameter_value().string_value
    self.right_camera_topic = self.get_parameter(
        'right_camera_topic').get_parameter_value().string_value
    self.cone_detections_topic_ = self.get_parameter(
        'cone_detections_topic').get_parameter_value().string_value
    self.heartbeat_topic_ = self.get_parameter(
        'heartbeat_topic').get_parameter_value().string_value
    self.update_rate_ = self.get_parameter(
        'update_rate').get_parameter_value().double_value

    distortion = self.get_parameter(
        'distortion').get_parameter_value().double_array_value

    intrinsics = self.get_parameter(
        'intrinsics').get_parameter_value().double_array_value

    print(intrinsics)
    print(distortion)

    intrinsics = np.array(intrinsics).reshape(3,3)
    distortion = np.array(distortion)

    print(intrinsics)
    print(distortion)
    # Create distortion maps:
    int_camera_mtx = np.array([[1109.0679923556017, 0.0, 725.8461303542504],
                      [0.0, 1106.1270765104496, 572.1637280271174],
                      [0.0, 0.0, 1.0]])
    distortion = np.array([[-0.26170828446607286, 0.109830602711457, -0.0009754103281966483, 0.0015736897048436397, -0.027459437237028456]])
    self.mapx, self.mapy = cv2.initUndistortRectifyMap(
        int_camera_mtx,\
        distortion,\
        None, None,\
        (1400,1080), cv2.CV_32FC1)

  def initSubscribers(self):
    """
    Initialize Subscribers

    left_cam_subscriber_ : 
      msg: sensor_msgs::Image, topic: kLeftImage

    right_cam_subscriber_ : 
      msg: sensor_msgs::Image, topic: kRightImage
    
    """
    self.left_cam_subscriber_ = self.create_subscription(
        Image,
        self.left_camera_topic,
        self.leftCameraCB,
        1)

    self.right_cam_subscriber_ = self.create_subscription(
        Image,
        self.right_camera_topic,
        self.rightCameraCB,
        1)

    # Call the left_cam_subscriber_ and right_cam_subscriber_ to prevent 
    # unused variable warnings
    self.left_cam_subscriber_
    self.right_cam_subscriber_


  def initPublishers(self):
    """
    Initialize Publisher

    cone_detections_publisher_ : 
      msg: utfr_msgs::ConeDetections, topic: kConeDetections
    
    heartbeat_publisher_:
      msg: utfr_msgs::Heartbeat, topic: kHeartbeat
    
    """
    self.cone_detections_publisher_ = \
        self.create_publisher(ConeDetections, self.cone_detections_topic_, 1)

    self.heartbeat_publisher_ = \
        self.create_publisher(Heartbeat, self.heartbeat_topic_, 1)

  def initServices(self):    
    """
    Initialize Services

    left_camera_client_ : triggers left camera shutter
      Trigger, topic: left_camera_node/trigger_image

    right_camera_client_ : triggers right camera shutter
      Trigger, topic: right_camera_node/trigger_image
    """

    self.left_camera_client_ = self.create_client(
        Trigger, 
        'left_camera_node/trigger_image')
    self.left_camera_request_ = Trigger.Request()

    self.right_camera_client_ = self.create_client(
        Trigger, 
        'right_camera_node/trigger_image')
    self.right_camera_request_ = Trigger.Request()

  def initTimers(self):
    """
    Initialize main update timer for timerCB.
    """
    #convert timer period in [ms] to timper period in [s]
    timer_period_s = self.update_rate_/1000
    self.timer_ = self.create_timer(timer_period_s, self.timerCB)

    # Call the timer_  to prevent unused variable warnings
    self.timer_

  def initHeartbeat(self):
    """
    Initialize parameters for heartbeat message
    heartbeat_.data: string: 
      name of node
    heartbeat_.update_rate: double: 
      update rate of node
    """
    self.heartbeat_ = Heartbeat()

    self.heartbeat_.module.data = "perception"
    self.heartbeat_.update_rate = self.update_rate_ 

    #TODO: Put these in a config/yaml file and read from there
    self.MIN_HEIGHT = 720
    self.MIN_WIDTH = 1020
    self.MAX_FRAME_RATE = 1.0
    if self.first_img_arrived_ == True:
      self.previous_left_img_ = self.left_img_
      self.previous_right_img_ = self.right_img_
      
  def publishHeartbeat(self):
    """
    Requires list of frames to check status of node.
    Publish heartbeat message.
    """
    self.heartbeat_.status = self.cameraStatus()
    self.heartbeat_.header.stamp = self.get_clock().now().to_msg()
    self.heartbeat_publisher_.publish(self.heartbeat_)




  def leftCameraCB(self, msg):
    """
    Callback function for left_cam_subscriber_
    """
    self.get_logger().warn(\
          'Recieved left camera message')
    try:
      self.left_img_ = \
          self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
      self.left_img_ = cv2.cvtColor(self.left_img_, cv2.COLOR_BayerRG2RGB)
      self.left_img_recieved_ = True

      if self.first_img_arrived_ == False:
        self.first_img_arrived_ = True
        
    except CvBridgeError as e:
      exception = "Perception::leftCameraCB: " + e
      self.get_logger().error(exception)

  def rightCameraCB(self, msg):
    """
    Callback function for right_cam_subscriber_
    """
    self.get_logger().warn(\
          'Recieved left camera message')
    try:
      self.right_img_ = \
          self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
      self.right_img_ = cv2.cvtColor(self.right_img_, cv2.COLOR_BayerRG2RGB)
      self.right_img_recieved_ = True
      
      if self.first_img_arrived_ == False:
        self.first_img_arrived_ = True

    except CvBridgeError as e:
      exception = "Perception::rightCameraCB: " + e
      self.get_logger().error(exception)

  def timerCB(self):
    """
    Main Processing loop for perception module.
    Send Asynchronous Trigger to both cameras at once, and process 
    incoming frames.
    """
    
    self.publishHeartbeat()

    detections_msg = ConeDetections()
    detections_msg.header.frame_id = "left_camera"

    
    trigger = Trigger.Request()

    self.future = self.left_camera_client_.call_async(trigger)
    self.future = self.right_camera_client_.call_async(trigger)
  

    if not self.left_img_recieved_:
      return

    if not self.right_img_recieved_:
      return
    
    undist_left = cv2.remap(
        self.left_img_,\
        self.mapx, self.mapy,\
        interpolation=cv2.INTER_NEAREST,\
        borderMode=cv2.BORDER_CONSTANT,\
        borderValue=(0, 0, 0, 0))

    undist_right = cv2.remap(
        self.right_img_,\
        self.mapx, self.mapy,\
        interpolation=cv2.INTER_NEAREST,\
        borderMode=cv2.BORDER_CONSTANT,\
        borderValue=(0, 0, 0, 0))

    #cv2.imshow('right_camera_undistort', undist_right)
    #cv2.waitKey(1)

    detections_msg.header.stamp = self.get_clock().now().to_msg()
    self.cone_detections_publisher_.publish(detections_msg)

    self.left_img_recieved_ = False
    self.right_img_recieved_ = False
  # Helper functions:

  def equalize_hist(self, image):
    colorimage_r = cv2.equalizeHist(image[:,:,0])
    colorimage_g = cv2.equalizeHist(image[:,:,1])
    colorimage_b = cv2.equalizeHist(image[:,:,2])
    
    colorimage_e = np.stack((colorimage_r, colorimage_g, colorimage_b), axis=2)
    return colorimage_e

  def save_frames_single_camera(self, image, camera):
    
    if camera == 'left':
      cv2.imwrite("left_camera" + str(self.saved_count) + '.png', image)
      self.saved_count += 1
    
    if camera == 'right':
      cv2.imwrite("right_camera" + str(self.saved_count) + '.png', image)
      self.saved_count += 1
    
  def save_frames_stereo(self, left_image, right_image):
    cv2.imwrite("left_camera_stereo_" + str(self.saved_count) + '.png', left_image)
    cv2.imwrite("right_camera_stereo" + str(self.saved_count) + '.png', right_image )
    self.saved_count += 1

  def hasVisualArtifacts(self, frame):
    """
    Returns true if the frame has visual artifacts, defined
    as contours found after blurring and thresholding the image.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return len(contours) > 0

  def frameChanged(self, frame, prev_frame):
    """
    Returns True if the frame has changed significantly from the previous frame.
    """

    diff = cv2.absdiff(frame, prev_frame)
    gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
    dilated = cv2.dilate(thresh, None, iterations=3)
    contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return len(contours) > 0
  
  def cameraStatus(self):
    """
    Takes in list of frames and returns status of node.
    - Checks if no frame has been received
    - Checks if frame is too small
    - Checks if FPS is too low
    - Checks for visual artifacts
    - Otherwise, node is active
    """
    if self.first_img_arrived_ == False:
      return 0 #UNINITIALIZED
    # Check if frame is too small
    if self.left_img_.shape[0] < self.MIN_HEIGHT or self.left_img_.shape[1] < self.MIN_WIDTH:
      return 2 #FATAL
    elif self.right_img_.shape[0] < self.MIN_HEIGHT or self.right_img_.shape[1] < self.MIN_WIDTH:
      return 2 #FATAL
    # Check if FPS is too low
    elif self.left_timestamp_ - self.right_timestamp_ > self.MAX_FRAME_RATE:
      return 2 #FATAL
    elif self.right_timestamp_ - self.left_timestamp_ > self.MAX_FRAME_RATE:
      return 2 #FATAL
    elif hasVisualArtifacts(self.left_img_):
      return 2 #FATAL
    elif hasVisualArtifacts(self.right_img_):
      return 2 #FATAL
    # Check if frame has changed significantly (stores a boolean)
    left_frame_changed = self.frameChanged(self.left_img_, self.previous_left_img_)
    right_frame_changed = self.frameChanged(self.right_img_, self.previous_right_img_)
    if left_frame_changed == False or right_frame_changed == False:
      #Update previous frames
      self.previous_left_img_ = self.left_img_
      self.previous_right_img_ = self.right_img_
      return 0 #UNINITIALIZED
    else:
      return 1 #ACTIVE
    
    

    def monitorCamera(self):
      """
      Monitor camera status and publish heartbeat message.
      Function or line below to be called in the main loop.
      """
      self.publishHeartbeat()


def main(args=None):
  rclpy.init(args=args)
  node = PerceptionNode()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == "__main__":
  main()