import cv2

# Specify the video file path
video_file = 'checkerboard.mp4'

# Create a VideoCapture object
cap = cv2.VideoCapture(video_file)

# Check if the VideoCapture object was successfully created
if not cap.isOpened():
    print("Error: Could not open video file.")
    exit()

# Create a list to store the frames
frames = []

# Loop over the video frames
while True:
    # Read the next frame
    ret, frame = cap.read()

    # Break the loop if there are no more frames
    if not ret:
        break

    # Add the frame to the list
    frames.append(frame)

# Release the VideoCapture object
cap.release()

# Save the frames to a file
with open('frames.pkl', 'wb') as f:
    pickle.dump(frames, f)