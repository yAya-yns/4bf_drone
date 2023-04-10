import numpy as np
import cv2 as cv
from jetcam.csi_camera import CSICamera
frame_width, frame_height = 640, 480
camera = CSICamera(width=frame_width, height=frame_height, capture_width=frame_width, capture_height=frame_height, capture_fps=30)


def update_image(change):
    image = change['new']
    image_widget.value = bgr8_to_jpeg(image)
    
# camera.observe(update_image, names='value')

# # option 1
# image = camera.read()
# camera.running=True
# print(camera.running)
# print(123123)



# cap = cv.VideoCapture(0)
# # Define the codec and create VideoWriter object
fourcc = cv.VideoWriter_fourcc(*'XVID')
out = cv.VideoWriter('rgb_output/output.avi', fourcc, 30.0, (frame_width,  frame_height))
while True: # camera.running:
    # ret, frame = cap.read()
    # if not ret:
    #     print("Can't receive frame (stream end?). Exiting ...")
    #     break
    # frame = cv.flip(frame, 0)
    # write the flipped frameq
    frame = image = camera.read()
    out.write(frame)
    cv.imshow('frame', frame)
    if cv.waitKey(1) == ord('q'):
        break
# Release everything if job is finished
# cap.release()
out.release()
cv.destroyAllWindows()