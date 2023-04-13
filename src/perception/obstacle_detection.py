import numpy as np
import cv2 as cv

def view(img, text=''):
    '''for quickly debugging and viewing image'''
    cv.imshow(text, img)
    cv.waitKey()
    # exit()

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

        processed_frame = obstacle_detection(frame)
        out.write(processed_frame)

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

def obstacle_detection(img, highlight_scale=1, alpha=2.5, beta=-20):
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
    norm_img = img

    # # adjust highlight
    # img = highlight_adjust(img, scale=highlight_scale, threshold=highlight_threshold)
    # img_highlight = img

    # adjust contrast and brightness
    img = cv.convertScaleAbs(img, alpha=alpha, beta=beta)
    img_contrast = img


    # adjust highlight
    img[img > highlight_threshold] = img[img > highlight_threshold] * highlight_scale
    img = np.uint8(img) 

    # filter out yellow
    img = filter_color(img, colors=['yellow'])

    # img = denoise(img)
    img_blur = cv.GaussianBlur(img, (5, 5), 2)

    return img_blur

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
    name = 'arena_video'
    input_name = f'src/perception/data/{name}.avi'
    output_name = input_name.replace(name, 'processed')
    
    process_video(input_name, output_name, display=True)