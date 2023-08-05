import cv2
import time
import numpy as np
from scipy.io import savemat

from .utils_display import DisplayHand
from .utils_mediapipe import MediaPipeHand
from .filters import OneEuroFilter, MovingAverageFilter, MovingMedianFilter

def video2keypoints(input_path, output_path="./data", display_captured=False, loop=False, static_hand=False, smoothing="OneEuroFilter",duration=-1):
    if isinstance(input_path,str):
        print("Using video: {}.".format(input_path))
    elif isinstance(input_path,int):
        print("Using Camera: {}".format(input_path))
    cap = cv2.VideoCapture(input_path)  # Read from .mp4 file
    Nframes = cap.get(cv2.CAP_PROP_FRAME_COUNT)
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if duration == -1:
        fps_video = cap.get(cv2.CAP_PROP_FPS)
    else: # Reformat recorded videos to real timestamp
        fps_video = Nframes/duration
    out = cv2.VideoWriter(output_path+".mp4", cv2.VideoWriter_fourcc(*'avc1'),fps_video, (int(frame_width), int(frame_height)))
    print("Video property: fps=", fps_video, "Nframes=", Nframes,"frame_width=", frame_width, "frame_height=", frame_height)
    timestampList = []
    keypoints = []
    handworld_keypoints = []
    transformations = []
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
                filter = MovingAverageFilter(window_size=10)
            param[0]['joint'] = filter(param[0]['joint'])
        elif smoothing == "MovingMedianFilter":
            if count == 0:
                filter = MovingMedianFilter(window_size=3)
            param[0]['joint'] = filter(param[0]['joint'])
        # Display keypoint
        annoated_img = img.copy()
        # disp.draw2d(annoated_img, param)
        disp.draw2d_default(annoated_img, param)
        if display_captured:
            cv2.imshow('img 2D',annoated_img)
            # Display 3D
            disp.draw3d(param, annoated_img)
            disp.vis.update_geometry(None)
            disp.vis.poll_events()
            disp.vis.update_renderer()
        if count < Nframes:
            annoated_img = cv2.flip(annoated_img, 1)
            out.write(annoated_img)
            keypoints.append(param[0]['joint'].copy())
            handworld_keypoints.append(param[0]['handworld_joint'].copy())
            transformations.append(param[0]['transformation'].copy())
            # timestampList.append(cap.get(cv2.CAP_PROP_POS_MSEC))
            timestampList.append(count*1000/fps_video)  # in milliseconds
            if count == Nframes-1:
                savemat(output_path+".mat", {'fps': fps_video, 'timestampList': timestampList,'keypoints': keypoints,'handword_keypoints':handworld_keypoints, 'transformations': transformations})
                print("mat saved!")
                if loop == False:
                    break
        count += 1
        key = cv2.waitKey(1)
        if key == 27:
            break
        if key == ord('r'):  # Press 'r' to reset camera view
            disp.camera.reset_view()

    pipe.pipe.close()
    cap.release()

if __name__ == "__main__":
    video2keypoints("../videos/1-1.mp4","./push_pull")