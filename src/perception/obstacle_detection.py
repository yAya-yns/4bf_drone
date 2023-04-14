import numpy as np
import cv2 as cv

def view(img, text=''):
    '''for quickly debugging and viewing image'''
    cv2.imshow(text, img)
    cv2.waitKey()
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
    # img_concat = cv2.hconcat([np.uint8(img) for img in imgs])
    img_concat = np.uint8(cv2.hconcat(imgs))
    # Display concatenated image
    cv2.imshow('Side-by-Side Image', img_concat)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def color_processing(img):
    highlight_scale=0.6
    highlight_threshold=230

    alpha=5
    beta=0
    '''
    highlight adjustment
    highlight_scale 0 - 1.0
    contrast adjustment
    alpha   # Contrast control (1.0-3.0)
    beta    # Brightness control (0-100)
    '''
    img = normalization(img)
    # # adjust highlight
    img = highlight_adjust(img, scale=highlight_scale, threshold=highlight_threshold)
    # adjust contrast and brightness
    img = contrast_n_brightness_adjust(img, alpha, beta)
    # # adjust highlight
    img = highlight_adjust(img, scale=highlight_scale, threshold=highlight_threshold)
    return img

def highlight_adjust(img, scale, threshold):
    img[img > threshold] = img[img > threshold] * scale
    img = np.uint8(img) 
    return img

def normalization(img):
    return cv2.normalize(img,None,0,255,cv2.NORM_MINMAX)

def contrast_n_brightness_adjust(img, alpha, beta):
    return cv2.convertScaleAbs(img, alpha=alpha, beta=beta)

def filter_color(img, colors):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img_out = np.zeros(img.shape)
    if 'yellow' in colors:
        lower_yellow1 = np.array([20, 150, 150])
        upper_yellow1 = np.array([30, 255, 255])


        lower_yellow2 = np.array([25, 50, 50])
        upper_yellow2 = np.array([35, 255, 255])

        # Create mask for yellow color range
        mask1 = cv2.inRange(hsv, lower_yellow1, upper_yellow1)
        mask2 = cv2.inRange(hsv, lower_yellow2, upper_yellow2)
        mask = cv2.bitwise_or(mask1, mask2)


        yellow = cv2.bitwise_and(img, img, mask=mask1)
        img_out += yellow
    if 'green' in colors:
        lower_green = np.array([30, 50, 50])
        upper_green = np.array([90, 255, 255])

        # Create a mask that isolates the green color from the rest of the image
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Apply the mask to the original image to get the green parts
        green = cv2.bitwise_and(img, img, mask=mask)
        img_out += green
    return np.uint8(normalization(img_out))

def process_video(input_name, output_name, display=False):
    cap = cv2.VideoCapture(input_name)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    
    # Define the output video codec and file name
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # out = cv2.VideoWriter(output_name, fourcc, fps, (w, h))
    
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
            cv2.imshow('Frame', processed_frame)
            key = cv2.waitKey(fps+3)
            if key == ord('q'):
                break

    # Release the video capture object and close all windows
    # out.release()
    cap.release()
    cv2.destroyAllWindows()
    # print(f'obj_detected video saved as: {output_name}')
    return True

def obstacle_detection(img : cv2.Mat, color_process=True):
    if color_process:
        img = color_processing(img)
    img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
    # view(img)
    img_blur = cv2.GaussianBlur(img, (5, 5), 2)
    # view(img_blur, text='blur')

    # color_test = np.zeros(img.shape)
    # color_test[:, :] = np.array([0, 1, 1])
    # # print(color_test)

    yellow = np.array([0, 1.0, 1.0])
    yellow /= np.linalg.norm(yellow)
    similarity = np.dot(img_blur, yellow) / np.linalg.norm(img_blur, axis=2)
    # print_img_stat(similarity, 'similarity')
    # view(similarity, text='similarity')

    _, thresh = cv2.threshold(similarity, 0.93, 1, cv2.THRESH_BINARY)
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
        return None, None, img_padded
    else:
        rect = cv2.minAreaRect(contours[index])
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        # print(rect)
        cv2.drawContours(img_padded, [box], 0, (255, 255, 255), 2)
        width = min(rect[1])    # assumption that most of the pillar is visible
        left_distance = rect[0][0]
        # print(width, left_distance)
        return width, left_distance, img_padded
    # filtered_contours = np.array(filtered_contours)
    # cnt_areas = np.array(cnt_areas)
    # print(f'total_area: {np.sum(cnt_areas)}')
    # print(cnt_areas / np.sum(cnt_areas))
    assert len(cx_position) == len(filtered_contours)
    contours = [x for _, x in sorted(zip(cx_position, filtered_contours))]

    diff = np.diff(sorted(cx_position))
    print(diff)
    if max(cnt_areas) > 0.6 * np.sum(cnt_areas) or np.max(diff) < 60:
        rect = cv2.minAreaRect(np.concatenate(contours))
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        print(rect)
        cv2.drawContours(img_padded, [box], 0, (255, 255, 255), 2)
        width = min(rect[1])    # assumption that most of the pillar is visible
        left_distance = rect[0][0]
        # print(width, left_distance)
        return width, left_distance, img_padded
    # elif len(cnt_areas) > 1:
    #     split_index = np.argmax(diff)
    #     cluster1, cluster2 = contours[0:split_index], contours[split_index:]
    #     area1, area2 = 




    return None, None, img_padded

if __name__ == "__main__":
    import os
    name = 'arena_video'
    input_name = f'src/perception/video_data/{name}.avi'
    output_name = input_name.replace(name, 'processed')

    dir_path = f'/Users/yefan/Documents/4bf_drone/src/perception/arena_video_pictures'
    # img_name = '0027.png'
    img_name = '0022.png'
    img_name = '0021.png'
    img_name = '0001.png'
    img_path = os.path.join(dir_path, img_name)

    assert os.path.isfile(img_path)

    img = cv2.imread(img_path, cv2.IMREAD_COLOR)
    # view(img, text='raw')

    # view(obstacle_detection(img)[2])
    
    process_video(input_name, output_name, display=True)