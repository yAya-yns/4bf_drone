#!/usr/bin/env python3
import sys
#sys.path.insert(0, "/usr/local/lib/python3.6/dist-packages/jetcam-0.0.0-py3.6.egg")
#print(sys.version)

import rospy
import cv2
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from jetcam.csi_camera import CSICamera

class DetNode:
    def __init__(self):
        rospy.init_node("monocular")

        # Configurable params:
        self.width_ = 640
        self.height_ = 480
        self.update_rate_ = 10
        self.visualize = True

        self.intrinsics_ = np.array([[411.838509092687, 0.0, 289.815738517589],
                                    [0.0, 546.791755994532, 278.771000222492],
                                    [0.0, 0.0, 1.0]])
        
        self.radial_ = np.array([-0.337287855055825,	0.117096291002566, 0, 0])  # no tangential distortion


        # Initialize variables:
        self.camera = CSICamera(\
                width= self.width_, height=self.height_, capture_width=self.width_, capture_height=self.height_, capture_fps=self.update_rate_)
        
        self.w_h_pair = (self.width_,self.height_)

        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.intrinsics_, self.radial_, self.w_h_pair, 1, self.w_h_pair)

        self.intrinsics_inv = np.linalg.inv(self.newcameramtx)

        self.od_pub_ = rospy.Publisher("/obj/pos", \
            Point, queue_size=10)
        self.timer_ = rospy.Timer(rospy.Duration(1.0/self.update_rate_), self.timer_cb)

    def timer_cb(self, event):
        img, ready = self.getImage()
        if not ready:
            print("image not yet ready")
            return
        img_undist = self.undistortImage(img)
        if self.visualize:
            cv2.imshow("monocular", img_undist)
            cv2.waitKey(1)

        msg = self.getObstacles(img_undist)

        self.od_pub_.publish(msg)
        return
    
    def getImage(self):
        try:
            img =  self.camera.read()
            img = cv2.rotate(img, cv2.ROTATE_180)
            ready = True
        except:
            img = None
            ready = False
        return img, ready

    def undistortImage(self, img):
        return cv2.undistort(img, self.intrinsics_ , self.radial_, None, self.newcameramtx)[self.roi[1]:self.roi[1]+self.roi[3], self.roi[0]:self.roi[0]+self.roi[2]]

    def getObstacles(self, img):
        msg = Point()

        #temp
        detection = True
        x = self.width_/2 #[px]
        depth = 2.0 #[m]

        if not detection:
            msg.z = -1.0        
            return msg
                
        y_centered = self.height_/2
        x_centered = self.width_/2
        img_hom = np.array([[x_centered], [y_centered], [1]])
        pos_hom = np.dot(self.intrinsics_inv, img_hom)
        
        pos =  pos_hom*depth / np.linalg.norm(pos_hom)

        msg.x = pos[2][0]
        msg.y = -pos[0][0]
        msg.z = -pos[1][0]

        print(msg)
        print("norm :" +  str(np.linalg.norm(pos)))
        print("")

        return msg

if __name__ == "__main__":
    det_node = DetNode()
    rospy.spin()

