import cv2

# Set video file name and fps
video_file = 'data/task4n5_undistorted.avi'
output_name = video_file[:-4] + "_clipped.avi"
fps = 30

# Set start and end times in seconds
start_time = 2 * 60 + 50
end_time = 3 * 60 + 30

# Open video file and get total number of frames
cap = cv2.VideoCapture(video_file)
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

# Calculate start and end frames based on fps and start/end times
start_frame = int(start_time * fps)
end_frame = int(end_time * fps)

# Check if start and end frames are within the range of the video
if start_frame > total_frames:
    start_frame = total_frames
if end_frame > total_frames:
    end_frame = total_frames

# Set the video output codec and filename
fourcc = cv2.VideoWriter_fourcc(*'XVID')

# Set the video output dimensions and fps
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
out = cv2.VideoWriter(output_name, fourcc, fps, (frame_width, frame_height))

# Read and write the video frames to the output file
for i in range(start_frame, end_frame):
    cap.set(cv2.CAP_PROP_POS_FRAMES, i)
    ret, frame = cap.read()
    if ret:
        out.write(frame)

# Release resources
cap.release()
out.release()
cv2.destroyAllWindows()