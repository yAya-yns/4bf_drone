#!/usr/bin/env python2.7
import rospy
import cv2
import numpy as np
import threading
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class DetNode:
    def __init__(self):
        print('starting od')
        rospy.init_node("det_node")

        # Configurable params:
        self.counter_threshold_ = 5
        self.inverse_disparity_threshold_ = 85.0
        self.pixels_to_check_ = 250
        self.downscale_factor_ = 7
        self.focal_ = 286.4

        self.num_disparities_ = 64
        self.min_disparities_ = 16
        self.block_size_ = 35
        self.visualize_ = False
        self.verbose_ = False

        # Initialize variables:
        self.disparity_image_ = None
        self.img_1_ = None
        self.img_2_ = None
        self.img_1_ready_ = False
        self.img_2_ready_ = False
        self.cam_1_info_ = None
        self.cam_2_info_ = None
        self.cam_1_map_y_ = None
        self.cam_1_map_x_ = None
        self.cam_2_map_y_ = None
        self.cam_2_map_x_ = None
        self.map_ready_ = False
        self.window_ = False
        self.counter_ = 0
        self.baseline_ = 0.064
        self.img_size_ = (800,848)
        self.cv_bridge_ = CvBridge()
        self.stereo_bm_ = cv2.StereoBM_create(
            numDisparities=self.num_disparities_, blockSize=self.block_size_)
        self.stereo_bm_.setMinDisparity(self.min_disparities_)
        self.window_avg_ = [-1,-1]
        self.window_counter_ = 0

        # Pubs, Subs and Timers
        rospy.Subscriber("/camera/fisheye1/image_raw", Image, \
                         self.cam_1_cb)
        rospy.Subscriber("/camera/fisheye1/camera_info", CameraInfo, \
                         self.cam_1_info_cb)
        rospy.Subscriber("/camera/fisheye2/image_raw", Image, \
                         self.cam_2_cb)
        rospy.Subscriber("/camera/fisheye2/camera_info", CameraInfo, \
                         self.cam_2_info_cb)
        
        self.od_pub_ = rospy.Publisher("/det/inview", \
            Float64, queue_size=10)
        self.timer_ = rospy.Timer(rospy.Duration(0.1), self.timer_cb)

    def cam_1_cb(self, msg):
      if self.map_ready_ == False:
        return
      img_distorted = self.cv_bridge_.imgmsg_to_cv2(msg, desired_encoding="passthrough")
      img_undistorted = cv2.remap(
          img_distorted,
          self.cam_1_map_x_,
          self.cam_1_map_y_,
          interpolation=cv2.INTER_LINEAR,
          borderMode=cv2.BORDER_CONSTANT,
      )
      # crop top and bottom based on DOWNSCALE_H
      orig_height = img_undistorted.shape[0]
      new_height = orig_height//self.downscale_factor_
      # take center of image of new height
      self.img_1_ =  np.fliplr(img_undistorted[int(orig_height/2) - new_height :int(orig_height/2), 250:700])
      # convert from mono8 to bgr8
      #self.img_1_ = cv2.cvtColor(img_undistorted, cv2.COLOR_GRAY2BGR)
      self.img_1_ready_ = True
        

    def cam_2_cb(self, msg):
      if self.map_ready_ == False:
        return
      img_distorted = self.cv_bridge_.imgmsg_to_cv2(msg, desired_encoding="passthrough")
      img_undistorted = cv2.remap(
          img_distorted,
          self.cam_2_map_x_,
          self.cam_2_map_y_,
          interpolation=cv2.INTER_LINEAR,
          borderMode=cv2.BORDER_CONSTANT,
      )
      # crop top and bottom based on DOWNSCALE_H
      orig_height = img_undistorted.shape[0]
      new_height = orig_height//self.downscale_factor_
      # take center of image of new height
      self.img_2_ = np.fliplr(img_undistorted[int(orig_height/2) - new_height :int(orig_height/2), 250:700])
      # convert from mono8 to bgr8
      #self.img_2_ = cv2.cvtColor(img_undistorted, cv2.COLOR_GRAY2BGR)
      self.img_2_ready_ = True
        

    def cam_1_info_cb(self, msg):
        if self.cam_1_info_ == None:
            self.cam_1_info_ = msg

    def cam_2_info_cb(self, msg):
        if self.cam_2_info_ == None:
            self.cam_2_info_ = msg

    def timer_cb(self, event):
        msg = Float64()
        msg.data = -1.0
        if (self.cam_1_info_ != None and self.cam_2_info_ != None) and \
            (self.map_ready_ == False):
            print("det_node::timer_cb: calling generate map")
            self.generateMap()

        if self.img_1_ready_ == False or self.img_2_ready_ == False:
            print("det_node::timer_cb: image pair not yet ready")
            
        else:
            self.generateDisparity()
            msg.data = self.processDisparity()
        self.od_pub_.publish(msg)  

        return
    
    def generateMap(self):
        k1 = np.array(self.cam_1_info_.K).reshape(3,3)
        d1 = np.array(self.cam_1_info_.D)
        k2 = np.array(self.cam_2_info_.K).reshape(3,3)
        d2 = np.array(self.cam_2_info_.D)
        T = np.array([self.baseline_, 0, 0]) # 64 mm baseline

        R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(\
            k1, d1, k2, d2, self.img_size_, R=np.eye(3), T=T)
        
        self.cam_1_map_x_, self.cam_1_map_y_ = \
            cv2.fisheye.initUndistortRectifyMap(\
                k1, d1, R1, P1, size=self.img_size_, m1type=cv2.CV_32FC1)
        
        self.cam_2_map_x_, self.cam_2_map_y_ = \
            cv2.fisheye.initUndistortRectifyMap(\
                k2, d2, R2, P2, size=self.img_size_, m1type=cv2.CV_32FC1) 
    
        self.map_ready_ = True

    def generateDisparity(self):
        self.disparity_image_ = self.stereo_bm_.compute(\
            self.img_2_, self.img_1_).astype(np.float32) / self.num_disparities_

        self.disparity_image_ =  self.disparity_image_

        if self.visualize_:
            disparity_color = cv2.applyColorMap(
                cv2.convertScaleAbs(self.disparity_image_, alpha=255 / 16.0), cv2.COLORMAP_JET)
            cv2.imshow("stereo", disparity_color)
            cv2.imshow("raw", self.img_2_)
            cv2.waitKey(1)



    def processDisparity(self):
        max_indexes = np.unravel_index(np.argsort(-self.disparity_image_, axis=None),\
                                       self.disparity_image_.shape)
        max_x = max_indexes[0][:self.pixels_to_check_]
        max_y = max_indexes[1][:self.pixels_to_check_]
        avg_max_disp = \
            np.average(self.disparity_image_[max_indexes][:self.pixels_to_check_])

        #print(opencv_image[max_indexes])

        if (avg_max_disp < 0):
            return -1.0

        avg_distance = 1000 / avg_max_disp
        
        avg_y = np.average(max_y)

        if self.verbose_:
            print("closest distance:")
            print(avg_distance)

            #print("y loc:")
            #print(np.average(max_y))

        #print(opencv_image.shape)
        if avg_distance < self.inverse_disparity_threshold_:
          if self.window_ == True:
              self.counter_ += 1
              self.window_avg_[self.window_counter_%2] = avg_y
              self.window_counter_ += 1

              if self.counter_ > self.counter_threshold_ :
                  return sum(self.window_avg_)/2
              
          else:
            self.window_ = True

        else:
            self.window_ = False
            self.counter_ = 0
        
        return -1.0

if __name__ == "__main__":
    det_node = DetNode()
    rospy.spin()

