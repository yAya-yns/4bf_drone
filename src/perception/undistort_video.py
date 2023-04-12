import numpy as np
import cv2 as cv


def crop_to_roi(img, roi):
    return img[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]]

def undistort(img, roi, K, new_K, dist_coeffs, need_crop=True):
    undistorted_img = cv.undistort(img, K, dist_coeffs, None, new_K)
    if need_crop:
        return crop_to_roi(undistorted_img, roi)
    return undistorted_img


def undistort_video(input_name, output_name, K, new_K, dist_coeffs, roi=None, need_crop=False):
    cap = cv.VideoCapture(input_name)
    w = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv.CAP_PROP_FPS))
    
    # Define the output video codec and file name
    fourcc = cv.VideoWriter_fourcc(*'XVID')
    if need_crop:
        w, h = roi[2], roi[3]
    out = cv.VideoWriter(output_name, fourcc, fps, (w, h))
    
    if not cap.isOpened():
        print("Error opening video file")

    # Loop over the frames in the video
    while cap.isOpened():
        # Read a frame from the video
        ret, frame = cap.read()

        # If the frame was not read successfully, break out of the loop
        if not ret:
            break

        undistorted_frame = undistort(frame, roi, K, new_K, dist_coeffs, need_crop=need_crop)
        out.write(undistorted_frame)

        # print((undistorted_frame.shape), (frame.shape))
        # xxx

        ###### uncomment below if you want to display the frame
        # cv.imshow('Frame', frame)
        # key = cv.waitKey(fps+3)
        # if key == ord('q'):
        #     break

    # Release the video capture object and close all windows
    out.release()
    cap.release()
    cv.destroyAllWindows()
    print("undistorted video saved as: " + output_name)


def main():
    K = np.array([
        [411.838509092687,	0,	289.815738517589],
        [0,	546.791755994532,	278.771000222492],
        [0,	0,	1],
        ])

    dist_coeffs = np.array([-0.337287855055825,	0.117096291002566, 0, 0])  # no tangential distortion
    '''
    after knowing the size of the frame, 
    new_K, roi = cv.getOptimalNewCameraMatrix(K, dist_coeffs, (w,h), 1, (w,h))

    new_K = 
        [[311.71820068   0.         288.97495611]
        [  0.         413.83319092 277.94188367]
        [  0.           0.           1.        ]]

    roi = (19, 44, 617, 394)
    '''


    # input_name = 'checkerboard.avi'
    input_name = 'checkerboard.avi'
    output_name = 'undist_' + input_name

    cap = cv.VideoCapture(input_name)
    w = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
    new_K, roi = cv.getOptimalNewCameraMatrix(K, dist_coeffs, (w,h), 1, (w,h))
    undistort_video(input_name, output_name, K, new_K, dist_coeffs, roi=roi, need_crop=True)



if __name__ == "__main__":
    main()