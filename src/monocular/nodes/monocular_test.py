# MIT License
# Copyright (c) 2019-2022 JetsonHacks

# Using a CSI camera (such as the Raspberry Pi Version 2) connected to a
# NVIDIA Jetson Nano Developer Kit using OpenCV
# Drivers for the camera and OpenCV are included in the base image

import cv2
import numpy as np
import time
import rospy
from geometry_msgs.msg import Point

class ObjectDetection:
    def __init__(self):
        rospy.init_node("monocular")
        # Configurable params:
        self.width_ = 640
        self.height_ = 480
        self.update_rate_ = 10
        self.const = 54.0
        self.visualize = True
        self.lower_yellow1 = np.array([20, 100, 100])
        self.upper_yellow1 = np.array([30, 255, 255])

        self.intrinsics_ = np.array([[411.838509092687, 0.0, 289.815738517589],
                                    [0.0, 546.791755994532, 278.771000222492],
                                    [0.0, 0.0, 1.0]])
        
        self.radial_ = np.array([-0.337287855055825,	0.117096291002566, 0, 0])  # no tangential distortion


        # Initialize variables:
        self.counter = 0
        
        self.w_h_pair = (self.width_,self.height_)

        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.intrinsics_, self.radial_, self.w_h_pair, 1, self.w_h_pair)

        self.intrinsics_inv = np.linalg.inv(self.newcameramtx)

        self.od_pub_ = rospy.Publisher("/obj/pos", \
            Point, queue_size=10)


    def process(self, img):
        img_undist = self.undistortImage(img)[:240,:]
        #img_undist = cv2.resize(img_undist, (320, 120), interpolation = cv2.INTER_NEAREST)
        img_filtered = self.filter_color(img_undist, ["yellow", "green"])
        width, x, img_padded, det_ready = self.obstacleDetection(img_filtered)
                       
        if self.visualize:
            #cv2.imshow("monocular", img_padded)
            #cv2.imshow("filtered", img_undist)
            #print(f'width: {width} \t x:{x}')
            self.counter += 1
            #cv2.imwrite('./img' + str(self.counter) + '.png', img_undist) 
            #cv2.waitKey(1)

        if not det_ready:
            return
        msg = self.getWorldPoint(width, x)
        #if self.visualize:
        print(msg)
        self.od_pub_.publish(msg)
        return
    
    def getImage(self):
        try:
            #img =  self.camera.read()
            ret_val, img = self.cap.read()
            img = cv2.rotate(img, cv2.ROTATE_180)
            ready = True
        except:
            img = None
            ready = False
        return img, ready

    def undistortImage(self, img):
        return cv2.undistort(img, self.intrinsics_ , self.radial_, None, self.newcameramtx)[self.roi[1]:self.roi[1]+self.roi[3], self.roi[0]:self.roi[0]+self.roi[2]]

    def getWorldPoint(self, width, x):
        msg = Point()

        depth = 83.3*(width**-0.939)

        detection = True

        if not detection:
            msg.z = -1.0        
            return msg
                
        y = self.height_/2
        img_hom = np.array([[x], [y], [1]])
        pos_hom = np.dot(self.intrinsics_inv, img_hom)
        
        pos =  pos_hom*depth / np.linalg.norm(pos_hom)

        msg.x = pos[2][0]
        msg.y = -pos[0][0]
        msg.z = -pos[1][0]

        #print(width)
        #print(msg)
        #print("norm :" +  str(np.linalg.norm(pos)))
        #print("")

        return msg

    
    def obstacleDetection(self, img, color_process=False):
        if color_process:
            img = self.color_processing(img)

        # img = cv2.colorChange(np.uint8(img), cv2.COLOR_HSV2BGR)
        # img = cv2.colorChange(img, cv2.COLOR_BGR2GRAY)
        # img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
        # # view(img)
        # img_blur = cv2.GaussianBlur(img, (5, 5), 2)
        # # view(img_blur, text='blur')

        # # color_test = np.zeros(img.shape)
        # # color_test[:, :] = np.array([0, 1, 1])
        # # # print(color_test)

        # yellow = np.array([0, 1.0, 1.0])
        # yellow /= np.linalg.norm(yellow)
        # similarity = np.dot(img_blur, yellow) / np.linalg.norm(img_blur, axis=2)
        # # print_img_stat(similarity, 'similarity')
        # # view(similarity, text='similarity')

        _, thresh = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY)
        #cv2.imshow("filtered", thresh)
        # view(thresh)
        kernel = np.ones((10,10),np.uint8)
        closing = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel)
        # view(opening)

        # print_img_stat(thresh)
        # view(thresh)
        pad = 10
        padded = cv2.copyMakeBorder(opening, pad, pad, 0, 0, cv2.BORDER_CONSTANT, value=0)
        img_padded = cv2.copyMakeBorder(img, pad, pad, 0, 0, cv2.BORDER_CONSTANT, value=0)
        # view(padded)

        thresh_blur = cv2.GaussianBlur(padded, (5, 5), 5)
        # view(thresh_blur)
        
        dst = cv2.Canny(np.uint8(thresh_blur) * 255, 50, 200, None, 5)
        # view(dst)
        #cv2.imshow("canny", dst)
        #return 10,11,dst,True
        contours, hierarchy = cv2.findContours(dst, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # print("Number of Contours found = " + str(len(contours)))
        # cv2.drawContours(img_padded, contours, -1, (255,255,255), 2)
        # view(img_padded)

        maxArea = 0
        cutoff_area = 500
        biggest = np.array([])
        index = None
        filtered_contours = []
        cnt_areas = []
        cx_position = []
        for i, cnt in enumerate(contours):  # Change - also provide index
            area = cv2.contourArea(cnt)
            # print(area)
            # cv2.drawContours(img_padded, contours, i, (255,255,255), 2)
            # view(img_padded)
            if area > cutoff_area:
                filtered_contours.append(cnt)
                cnt_areas.append(area)
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt,0.02*peri, True)
                M = cv2.moments(cnt)
                cX = int(M["m10"] / M["m00"])
                # cY = int(M["m01"] / M["m00"])
                # print(cX)
                cx_position.append(cX)
                # if area > maxArea and len(approx) == 4:
                if area > maxArea:  # !!! DANGEROUS !!!
                    biggest = approx
                    maxArea = area
                    index = i  # Also save index to contour
        # print(f'index: {index}')
        if index is None:
            return None, None, img_padded, False
        else:
            x, y, w, h = cv2.boundingRect(contours[index])
            # box = cv2.boxPoints(rect)
            # box = np.int0(box)
            # # print(rect)
            # cv2.drawContours(img_padded, [box], 0, (255, 255, 255), 2)
            # width = min(rect[1])    # assumption that most of the pillar is visible
            left_distance = x
            cv2.rectangle(img_padded, (x, y), (x+w, y+h), (255, 255, 255), 2)
            # print(width, left_distance)None
            return w, left_distance, img_padded, True

    def filter_color(self, img, colors):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_out = np.zeros(img.shape)
        if 'yellow' in colors:
            # Create mask for yellow color range
            mask = cv2.inRange(hsv, self.lower_yellow1, self.upper_yellow1)
            yellow = cv2.bitwise_and(img, img, mask=mask)
            img_out += yellow
        if 'green' in colors:
            lower_green = np.array([30, 125, 125])
            upper_green = np.array([90, 255, 255])

            # Create a mask that isolates the green color from the rest of the image
            mask = cv2.inRange(hsv, lower_green, upper_green)

            # Apply the mask to the original image to get the green parts
            green = cv2.bitwise_and(img, img, mask=mask)
            img_out += green
        return np.uint8((img_out))
    
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=640,
    capture_height=480,
    display_width=640,
    display_height=480,
    framerate=30,
    flip_method=2,
):
    return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


def show_camera():
    window_title = "CSI Camera"
    
    od = ObjectDetection()
    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    print(gstreamer_pipeline(flip_method=0))
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
    if video_capture.isOpened():
        try:
            #window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
            while True:
                ret_val, frame = video_capture.read()
                od.process(frame)
                # Check to see if the user closed the window
                # Under GTK+ (Jetson Default), WND_PROP_VISIBLE does not work correctly. Under Qt it does
                # GTK - Substitute WND_PROP_AUTOSIZE to detect if window has been closed by user
               #if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                    #cv2.imshow(window_title, frame)
                #else:
                #    break 
                #keyCode = cv2.waitKey(10) & 0xFF
                # Stop the program on the ESC key or 'q'
                #if keyCode == 27 or keyCode == ord('q'):
                #    break
        finally:
            video_capture.release()
            cv2.destroyAllWindows()
    else:
        print("Error: Unable to open camera")


if __name__ == "__main__":
    show_camera()
