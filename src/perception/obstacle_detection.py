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

        # print(processed_frame.shape)
        # print((w,h))
        # xxxx

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
    '''
    Plan:

    step 1: detect pillars, return bounding box coordinate

    step 2: identify pillars
    
    '''
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
    # img = cv.equalizeHist(img)

    # # adjust highlight
    # img = highlight_adjust(img, scale=highlight_scale, threshold=highlight_threshold)
    # img_highlight = img

    # adjust contrast and brightness
    # view(img)
    # img = cv.convertScaleAbs(img, alpha=alpha, beta=beta)
    # view(img)
    # img_contrast = img


    # adjust highlight
    # view(img)
    # img[img > highlight_threshold] = img[img > highlight_threshold] * highlight_scale
    # img = np.uint8(img) 
    # view(img)
    # filter out yellow
    # img = filter_color(img, colors=['yellow'])

    # view(img)
    img_blur = cv.GaussianBlur(img, (5, 5), 2)
    # view(img_blur)

    # img = denoise(img)
    color_test = np.zeros(img.shape)
    color_test[:, :] = np.array([0, 1, 1])
    yellow = np.array([0, 1.0, 1.0])
    yellow /= np.linalg.norm(yellow)
    # print(color_test)
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
    # print_img_stat(np.uint8(thresh_blur))
    # view(thresh_blur)
    # print(thresh_blur.dtype)

    
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

# def highlight_adjust(img, scale, threshold):
#     img[img > threshold] = img[img > threshold] * scale
#     img = np.uint8(img) 
#     return img

def normalization(img):
    return cv.normalize(img,None,0,255,cv.NORM_MINMAX)


# def contrast_n_brightness_adjust(img, alpha, beta):
#     return cv.convertScaleAbs(img, alpha=alpha, beta=beta)

def filter_color(img, colors):
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    img_out = np.zeros(img.shape)
    if 'yellow' in colors:
        # Define lower and upper HSV color range for yellow
        lower_yellow1 = np.array([20, 150, 150])
        upper_yellow1 = np.array([30, 255, 255])


        lower_yellow2 = np.array([25, 50, 50])
        upper_yellow2 = np.array([35, 255, 255])

        # compare_img([np.full((30, 30, 3), lower_yellow1), np.full((30, 30, 3), upper_yellow1), 
        #              np.full((30, 30, 3), lower_yellow2), np.full((30, 30, 3), upper_yellow2)])
        # xxx

        # Create mask for yellow color range
        mask1 = cv.inRange(hsv, lower_yellow1, upper_yellow1)
        mask2 = cv.inRange(hsv, lower_yellow2, upper_yellow2)
        mask = cv.bitwise_or(mask1, mask2)

        # Apply mask to extract yellow regions from original image
        # compare_img([img, cv.bitwise_and(img, img, mask=mask), 
        #              cv.bitwise_and(img, img, mask=mask1), cv.bitwise_and(img, img, mask=mask2)])
        # xxx

        yellow = cv.bitwise_and(img, img, mask=mask1)
        img_out += yellow
    if 'green' in colors:
        lower_green = np.array([30, 50, 50])
        upper_green = np.array([90, 255, 255])

        # Create a mask that isolates the green color from the rest of the image
        mask = cv.inRange(hsv, lower_green, upper_green)

        # Apply the mask to the original image to get the green parts
        green = cv.bitwise_and(img, img, mask=mask)
        img_out += green
    return np.uint8(normalization(img_out))

# def findContours(img, kernel_size):
#     kernel = np.ones(kernel_size, np.uint8)
#     img = cv.morphologyEx(img, cv.MORPH_CLOSE, kernel)
#     contours, hierarchy = cv.findContours(img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

#     min_width = 20
#     min_height = 50

#     for contour in contours:
#         x,y,w,h = cv.boundingRect(contour)
#         aspect_ratio = float(w)/h
#         if aspect_ratio < 0.5 and w > min_width and h > min_height:
#             img = cv.rectangle(img, (x,y), (x+w,y+h), (0,255,0), 2)
#     return img


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