import numpy as np
import cv2 as cv

'''
usage: 

from undistort import undistort_imx219
undistorted_frame = undistort_imx219(frame, need_crop=True)

# Note: if you crop and want to save the video, remember to change the videoWriter's input_size


'''

def crop_to_roi(img, roi):
    return img[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]]

def undistort(img, roi, K, new_K, dist_coeffs, need_crop=True):
    undistorted_img = cv.undistort(img, K, dist_coeffs, None, new_K)
    if need_crop:
        return crop_to_roi(undistorted_img, roi)
    return undistorted_img



def undistort_imx219(img, need_crop=True):
    # Step 1: Getting internal matrix and distortion coefficient. w,h for the camera
    w, h = 640, 480
    K = np.array([
        [411.838509092687,	0,	289.815738517589],
        [0,	546.791755994532,	278.771000222492],
        [0,	0,	1],
        ])

    dist_coeffs = np.array([-0.337287855055825,	0.117096291002566, 0, 0])  # no tangential distortion


    # Step 2: Getting width and h of img frame
    '''
    # input_name = 'checkerboard.avi'
    # cap = cv.VideoCapture(input_name)
    # w = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
    # h = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))

    ## or 
    img = cv2.imread('path/to/image.jpg')
    # Get the size of the frame
    h, w, _ = img.shape
    '''


    # Step 3: Calculate new_K and region of interest (roi) given the width and height

    # new_K, roi = cv.getOptimalNewCameraMatrix(K, dist_coeffs, (w,h), 1, (w,h))
    # print(new_K, roi)
    # xx
    # in this case, we have calculated previously.
    new_K = np.array([[311.71820068,   0.,         288.97495611],
            [  0. ,        413.83319092, 277.94188367],
            [  0. ,          0.,           1.        ]])

    roi = (19, 44, 617, 394)

    # Step 4: Undistort the image

    return undistort(img, roi, K, new_K, dist_coeffs, need_crop=need_crop)
