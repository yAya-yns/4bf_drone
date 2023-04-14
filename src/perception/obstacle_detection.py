import numpy as np
import cv2 as cv

def view(img, text=''):
    '''for quickly debugging and viewing image'''
    cv.imshow(text, img)
    cv.waitKey()
    # exit()

def print_img_stat(img : np.ndarray, text=''):
    print(text)
    print(f'dtype: {type(img.dtype)}')
    print(f'shape: {img.shape}')
    print(f'max: {np.max(img)}')
    print(f'min: {np.min(img)}')
    print(f'average: {np.average(img)}')
    # print(f'var: {np.var(img, axis=-1)}')

def compare_img(imgs):
    # img_concat = cv.hconcat([np.uint8(img) for img in imgs])
    img_concat = np.uint8(cv.hconcat(imgs))
    # Display concatenated image
    cv.imshow('Side-by-Side Image', img_concat)
    cv.waitKey(0)
    cv.destroyAllWindows()


def process_video(input_name, output_name, display=False):
    cap = cv.VideoCapture(input_name)
    w = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv.CAP_PROP_FPS))
    
    # Define the output video codec and file name
    fourcc = cv.VideoWriter_fourcc(*'XVID')
    out = cv.VideoWriter(output_name, fourcc, fps, (w, h))
    
    assert cap.isOpened(), "Error opening video file"
    

    # Loop over the frames in the video
    while cap.isOpened():
        # Read a frame from the video
        ret, frame = cap.read()

        # If the frame was not read successfully, break out of the loop
        if not ret:
            break

        width, left_dist, processed_frame = obstacle_detection(frame)
        # out.write(processed_frame)

        if display:
            cv.imshow('Frame', processed_frame)
            key = cv.waitKey(fps+3)
            if key == ord('q'):
                break

    # Release the video capture object and close all windows
    out.release()
    cap.release()
    cv.destroyAllWindows()
    print(f'obj_detected video saved as: {output_name}')
    return True

def obstacle_detection(img : cv.Mat, highlight_scale=1, alpha=2.5, beta=-20):
    img = cv.normalize(img, None, 0, 255, cv.NORM_MINMAX)
    # view(img)
    img_blur = cv.GaussianBlur(img, (5, 5), 2)
    # view(img_blur)

    # color_test = np.zeros(img.shape)
    # color_test[:, :] = np.array([0, 1, 1])
    # # print(color_test)

    yellow = np.array([0, 1.0, 1.0])
    yellow /= np.linalg.norm(yellow)
    similarity = np.dot(img_blur, yellow) / np.linalg.norm(img_blur, axis=2)
    print_img_stat(similarity, 'similarity')
    # view(similarity)

    _, thresh = cv.threshold(similarity, 0.93, 1, cv.THRESH_BINARY)
    # print_img_stat(thresh)
    # view(thresh)
    pad = 10
    padded = cv.copyMakeBorder(thresh, pad, pad, 0, 0, cv.BORDER_CONSTANT, value=0)
    img_padded = cv.copyMakeBorder(img, pad, pad, 0, 0, cv.BORDER_CONSTANT, value=0)
    # view(padded)

    thresh_blur = cv.GaussianBlur(padded, (5, 5), 5)
    # view(thresh_blur)
    
    dst = cv.Canny(np.uint8(thresh_blur) * 255, 50, 200, None, 5)
    # view(dst)

    contours, hierarchy = cv.findContours(dst, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    print("Number of Contours found = " + str(len(contours)))
    # cv.drawContours(img_padded, contours, -1, (255,255,255), 2)
    # view(img_padded)

    maxArea = 0
    biggest = np.array([])
    index = None
    for i, cnt in enumerate(contours):  # Change - also provide index
        area = cv.contourArea(cnt)
        # print(area)
        # cv.drawContours(img_padded, contours, i, (255,255,255), 2)
        # view(img_padded)
        if area > 500:
            peri = cv.arcLength(cnt, True)
            approx = cv.approxPolyDP(cnt,0.02*peri, True)
            # if area > maxArea and len(approx) == 4:
            if area > maxArea:  # !!! DANGEROUS !!!
                biggest = approx
                maxArea = area
                index = i  # Also save index to contour
    print(f'index: {index}')

    if index is not None:
        rect = cv.minAreaRect(contours[index])
        box = cv.boxPoints(rect)
        box = np.int0(box)
        print(rect)
        cv.drawContours(img_padded, [box], 0, (255, 255, 255), 2)
        width = min(rect[1])    # assumption that most of the pillar is visible
        left_distance = rect[0][0]
        # print(width, left_distance)
        # view(img)

        return width, left_distance, img_padded
    else: 
        return None, None, img_padded

if __name__ == "__main__":
    import os
    name = 'arena_video'
    input_name = f'src/perception/video_data/{name}.avi'
    output_name = input_name.replace(name, 'processed')

    dir_path = f'/Users/yefan/Documents/4bf_drone/src/perception/arena_video_pictures'
    img_name = '0032.png'
    img_path = os.path.join(dir_path, img_name)

    assert os.path.isfile(img_path)

    img = cv.imread(img_path, cv.IMREAD_COLOR)
    # view(img, text='raw')

    # obstacle_detection(img)
    
    process_video(input_name, output_name, display=True)