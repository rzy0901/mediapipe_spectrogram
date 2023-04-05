import cv2
import time
import numpy as np

from utils.utils_display import DisplayHand, DisplayBody, DisplayHolistic
from utils.utils_mediapipe import MediaPipeHand, MediaPipeBody, MediaPipeHolistic
from scipy.io import savemat

loop = True
input = "./videos/1-1.mp4"
cap = cv2.VideoCapture(input) # Read from .mp4 file
fps_video = cap.get(cv2.CAP_PROP_FPS)
frame_width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
out = cv2.VideoWriter('./output.mp4',cv2.VideoWriter_fourcc(*'MP4V'), fps_video, (int(frame_width),int(frame_height)))
Nframes = cap.get(cv2.CAP_PROP_FRAME_COUNT)
timestampList = []
keypoints = []
print("Video property: fps=", fps_video, "frames=", Nframes,"frame_width=",frame_width,"frame_height=",frame_height)
intrin = {
    'fx': frame_width*0.9, # Approx 0.7w < f < w https://www.learnopencv.com/approximate-focal-length-for-webcams-and-cell-phone-cameras/
    'fy': frame_width*0.9,
    'cx': frame_width*0.5, # Approx center of image
    'cy': frame_height*0.5,
    'width': frame_width,
    'height': frame_height,
}

pipe = MediaPipeHand(static_image_mode=False, max_num_hands=2, intrin=intrin)
disp = DisplayHand(draw3d=True, draw_camera=True, max_num_hands=2, intrin=intrin)
count = 0
prev_time = time.time()
while cap.isOpened():
    ret, img = cap.read()
    count += 1
    if not ret:
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0) # Loop back
        ret, img = cap.read()
        # break

    # Flip image for 3rd person view
    img = cv2.flip(img, 1)
    # To improve performance, optionally mark image as not writeable to pass by reference
    img.flags.writeable = False
    # Feedforward to extract keypoint
    param = pipe.forward(img)
    # Compute FPS
    curr_time = time.time()
    fps = 1/(curr_time-prev_time)
    param[0]['fps'] = fps
    prev_time = curr_time    
    img.flags.writeable = True
    # Display keypoint
    cv2.imshow('img 2D', disp.draw2d(img, param))
    if count <= Nframes:
        out.write(img)
        temp = param[0]['joint']
        keypoints.append(temp.copy())
        timestampList.append(cap.get(cv2.CAP_PROP_POS_MSEC))
        if count == Nframes:
            savemat('./data.mat',{'fps':fps_video,'timestampList':timestampList,'keypoints':keypoints})
            print("mat saved!")
            if loop == False:
                break
    # Display 3D
    disp.draw3d(param, img)
    disp.vis.update_geometry(None)
    disp.vis.poll_events()
    disp.vis.update_renderer()    
    key = cv2.waitKey(1)
    if key==27:
        break
    if key==ord('r'): # Press 'r' to reset camera view
        disp.camera.reset_view()

pipe.pipe.close()
cap.release()
