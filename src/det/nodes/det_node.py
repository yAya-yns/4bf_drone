#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import threading
from std_msgs.msg import Float64
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge

class DetNode:
    def __init__(self):
        print('starting od')
        rospy.init_node("det_node")

        # Initialize variables:
        self.disparity_image_ = None
        self.window_ = False
        self.counter_ = 0
        self.counter_threshold_ = 5
        self.distance_threshold_ = 1.0
        self.pixels_to_check_ = 50
        self.cv_bridge_ = CvBridge()

        # Pubs, Subs and Timers
        self.od_pub_ = rospy.Publisher("/det/inview", \
            Float64, queue_size=10)
        self.local_pose_sub_ = rospy.Subscriber("/stereo/disparity", \
            DisparityImage, callback = self.disparity_cb)
        self.timer_ = rospy.Timer(rospy.Duration(0.1), self.timer_cb)

    def disparity_cb(self, msg):
        self.disparity_image_ = msg
        return

    def timer_cb(self, event):
        msg = Float64()
        msg.data = -1.0
        
        if self.disparity_image_ == None:
            print("det_node::timer_cb: Dispairty image not yet recieved")
            
        else:
            msg.data = self.processDisparity()

        print(msg)
        self.od_pub_.publish(msg)  

        return
        
    def processDisparity(self):
        opencv_image = \
            self.cv_bridge_.imgmsg_to_cv2(self.disparity_image_.image, \
                                          desired_encoding='passthrough')
        
        cv2.imshow('image',opencv_image)
        cv2.waitKey(1)

        

        #print(opencv_image)

        dummpy = np.array([(1, 2), \
                           (4, 3)])

        max_indexes = np.unravel_index(np.argsort(-opencv_image, axis=None),\
                                       opencv_image.shape)
        max_x = max_indexes[0][:self.pixels_to_check_]
        max_y = max_indexes[1][:self.pixels_to_check_]
        avg_max_disp = \
            np.average(opencv_image[max_indexes][:self.pixels_to_check_])

        #print(opencv_image[max_indexes])

        if (avg_max_disp < 0):
            return -1.0

        avg_distance = \
            self.disparity_image_.f *self.disparity_image_.T / avg_max_disp
        
        avg_y = np.average(max_y)

        print("closest distance:")
        print(avg_distance)

        print("y loc:")
        print(np.average(max_y))

        #print(opencv_image.shape)
        if avg_distance < self.distance_threshold_:
          if self.window_ == True:
              self.counter_ += 1

              if self.counter_ > self.counter_threshold_ :
                  return avg_y
              
          else:
            self.window_ = True

        else:
            self.window_ = False
            self.counter_ = 0
        
        return -1.0

if __name__ == "__main__":
    det_node = DetNode()
    rospy.spin()

