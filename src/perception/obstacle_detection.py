import numpy as np
import cv2 as cv


def compare_img(img1, img2):
    height, width, _ = img1.shape
    img2 = cv.resize(img2, (width, height))

    # Concatenate images horizontally
    img_concat = cv.hconcat([img1, img2])

    # Display concatenated image
    cv.imshow('Side-by-Side Image', img_concat)
    cv.waitKey(0)
    cv.destroyAllWindows()


def obj_obstacle_detection_video(input_name, output_name):
    cap = cv.VideoCapture(input_name)
    w = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv.CAP_PROP_FPS))
    
    # Define the output video codec and file name
    fourcc = cv.VideoWriter_fourcc(*'XVID')
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

        processed_frame = detect_object(frame)
        out.write(processed_frame)


        ###### uncomment below if you want to display the frame
        # cv.imshow('Frame', frame)
        # key = cv.waitKey(fps+3)
        # if key == ord('q'):
        #     break

    # Release the video capture object and close all windows
    out.release()
    cap.release()
    cv.destroyAllWindows()
    print("obj_detected video saved as: " + output_name)
    return True

def detect_object(img):
    cv.imshow('Frame', img)

    detected_img = img
    
    compare_img(img, detected_img)
    xxxx
    return detected_img


def main():

    input_name = 'data/arena_video.avi'
    output_name = input_name[:-4] + "_detected.avi"
    
    obj_obstacle_detection_video(input_name, output_name)



    return True




if __name__ == "__main__":
    main()