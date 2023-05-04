import cv2
import time
import numpy as np

from utils.utils_display import DisplayHand
from utils.utils_mediapipe import MediaPipeHand
from scipy.io import savemat
from utils.filters import OneEuroFilter, MovingAverageFilter, MovingMedianFilter, LowPassFilter

display_captured = True            # Show the captured result
loop = True                        # Loop the input video
static_hand = True                  # Fix hand position to reduce jitter if knowing hand does the action in place.
# Note: smoothing only affects on one hand.
smoothing = "OneEuroFilter"         # "", "OneEuroFilter", "MovingAverageFilter", "MovingMedianFilter", "LowPassFilter"    
print("Smoothing Method used: {}.".format(smoothing))
input = "./videos/2.mp4"
cap = cv2.VideoCapture(input)  # Read from .mp4 file
fps_video = cap.get(cv2.CAP_PROP_FPS)
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
out = cv2.VideoWriter('./output.mp4', cv2.VideoWriter_fourcc(*'avc1'),fps_video, (int(frame_width), int(frame_height)))
Nframes = cap.get(cv2.CAP_PROP_FRAME_COUNT)
timestampList = []
keypoints = []
transformations = []
print("Video property: fps=", fps_video, "frames=", Nframes,
      "frame_width=", frame_width, "frame_height=", frame_height)
intrin = {
    # Approx 0.7w < f < w https://www.learnopencv.com/approximate-focal-length-for-webcams-and-cell-phone-cameras/
    'fx': frame_width*0.9,
    'fy': frame_width*0.9,
    'cx': frame_width*0.5,  # Approx center of image
    'cy': frame_height*0.5,
    'width': frame_width,
    'height': frame_height,
}

pipe = MediaPipeHand(static_image_mode=False, max_num_hands=2, intrin=intrin)
disp = DisplayHand(draw3d=display_captured, draw_camera=True,
                   max_num_hands=2, intrin=intrin)
count = 0
prev_time = time.time()
while cap.isOpened():
    ret, img = cap.read()
    if not ret:
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Loop back
        ret, img = cap.read()
        # break
    # Flip image for 3rd person view
    img = cv2.flip(img, 1)
    # To improve performance, optionally mark image as not writeable to pass by reference
    img.flags.writeable = False
    # Feedforward to extract keypoint
    param = pipe.forward(img, static_hand=static_hand)
    # Compute FPS
    curr_time = time.time()
    fps = 1/(curr_time-prev_time)
    param[0]['fps'] = fps
    prev_time = curr_time
    if smoothing == "OneEuroFilter":
        if count == 0:
            # filter = OneEuroFilter(x0=param[0]['joint'], dx0=0.0, t0 = 0,min_cutoff=0.004, beta=0.7, d_cutoff=1.0)
            filter = OneEuroFilter(x0=param[0]['joint'], dx0=0.0, t0 = 0,min_cutoff=5, beta=0.1, d_cutoff=20)
        else:
            param[0]['joint'] = filter(param[0]['joint'], t = count*1/fps_video)
    elif smoothing == "MovingAverageFilter":
        if count == 0:
            filter = MovingAverageFilter(window_size=3)
        param[0]['joint'] = filter(param[0]['joint'])
    elif smoothing == "MovingMedianFilter":
        if count == 0:
            filter = MovingMedianFilter(window_size=3)
        param[0]['joint'] = filter(param[0]['joint'])
    elif smoothing == "LowPassFilter":
        if count == 0:
            filter = LowPassFilter(fs=fps_video,low_cut=5,order=5)
        else:
            param[0]['joint'] = filter(param[0]['joint'])
    img.flags.writeable = True
    # Display keypoint
    if display_captured:
        cv2.imshow('img 2D', disp.draw2d(img, param))
    if count < Nframes:
        out.write(img)
        keypoints.append(param[0]['joint'].copy())
        transformations.append(param[0]['transformation'].copy())
        # timestampList.append(cap.get(cv2.CAP_PROP_POS_MSEC))
        timestampList.append(count*1000/fps_video)  # in milliseconds
        if count == Nframes-1:
            savemat('./data.mat', {'fps': fps_video, 'timestampList': timestampList,
                    'keypoints': keypoints, 'transformations': transformations})
            print("mat saved!")
            if loop == False:
                break
    # Display 3D
    if display_captured:
        disp.draw3d(param, img)
        disp.vis.update_geometry(None)
        disp.vis.poll_events()
        disp.vis.update_renderer()
    count += 1
    key = cv2.waitKey(1)
    if key == 27:
        break
    if key == ord('r'):  # Press 'r' to reset camera view
        disp.camera.reset_view()

pipe.pipe.close()
cap.release()
