import cv2
import numpy as np

# save frames
save = False
save_dir = "/home/jesse/Desktop/saved_video_frames/"
frame_count = 0
# Create a VideoCapture object and read from input file
# If the input is the camera, pass 0 instead of the video file name
cap = cv2.VideoCapture('/home/jesse/Desktop/aruco_night_10_16_17.avi')

# Check if camera opened successfully
if (cap.isOpened()== False):
  print("Error opening video stream or file")

# Read until video is completed
while(cap.isOpened()):
  # Capture frame-by-frame
  ret, frame = cap.read()
  if ret == True:

    # increment the frame counter
    frame_count = frame_count + 1

    # Display the resulting frame
    cv2.imshow('Frame',frame)

    # Press Q on keyboard to  exit
    if cv2.waitKey(25) & 0xFF == ord('q'):
      save = False
      break

    # Press S to save start saving frames
    if cv2.waitKey(25) & 0xFF == ord('s'):
        save = True
        print("Saving Frames...")


    # write the current frame to file
    if save == True:
        # convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.imwrite(save_dir + "frame_" + str(frame_count) + ".jpg", gray)

  # Break the loop
  else:
    break

# When everything done, release the video capture object
cap.release()

# Closes all the frames
cv2.destroyAllWindows()
