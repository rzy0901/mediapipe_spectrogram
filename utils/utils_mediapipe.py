###############################################################################
### adapted from https://github.com/ntu-rris/google-mediapipe
###############################################################################

import cv2
import numpy as np
import mediapipe as mp


# Define default camera intrinsic
img_width  = 640
img_height = 480
intrin_default = {
    'fx': img_width*0.9, # Approx 0.7w < f < w https://www.learnopencv.com/approximate-focal-length-for-webcams-and-cell-phone-cameras/
    'fy': img_width*0.9,
    'cx': img_width*0.5, # Approx center of image
    'cy': img_height*0.5,
    'width': img_width,
    'height': img_height,
}


class MediaPipeHand:
    def __init__(self, static_image_mode=True, max_num_hands=2,
        model_complexity=1, intrin=None):
        self.max_num_hands = max_num_hands
        if intrin is None:
            self.intrin = intrin_default
        else:
            self.intrin = intrin

        # Access MediaPipe Solutions Python API
        mp_hands = mp.solutions.hands
        # help(mp_hands.Hands)

        # Initialize MediaPipe Hands
        # static_image_mode:
        #   For video processing set to False: 
        #   Will use previous frame to localize hand to reduce latency
        #   For unrelated images set to True: 
        #   To allow hand detection to run on every input images
        
        # max_num_hands:
        #   Maximum number of hands to detect

        # model_complexity:
        #   Complexity of the hand landmark model: 0 or 1.
        #   Landmark accuracy as well as inference latency generally
        #   go up with the model complexity. Default to 1.
        
        # min_detection_confidence:
        #   Confidence value [0,1] from hand detection model
        #   for detection to be considered successful
        
        # min_tracking_confidence:
        #   Minimum confidence value [0,1] from landmark-tracking model
        #   for hand landmarks to be considered tracked successfully, 
        #   or otherwise hand detection will be invoked automatically on the next input image.
        #   Setting it to a higher value can increase robustness of the solution, 
        #   at the expense of a higher latency. 
        #   Ignored if static_image_mode is true, where hand detection simply runs on every image.

        self.pipe = mp_hands.Hands(
            static_image_mode=static_image_mode,
            max_num_hands=max_num_hands,
            model_complexity=model_complexity,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)

        # Define hand parameter
        self.param = []
        for i in range(max_num_hands):
            p = {
                'count'   : -1,               # Frame index of hand, increasing 1 as pipe.forward().
                'keypt_raw': np.zeros((21,3)), # Raw 2.5D normalized keypt in image coordinate (between 0 and 1)_
                'keypt'   : np.zeros((21,2)), # 2D keypt in image coordinate (pixel)
                'joint'   : np.zeros((21,3)), # 3D joint in camera coordinate (m)
                'handworld_joint': np.zeros((21,3)), # Raw 3D joint in hand coordinate (m)
                'class'   : None,             # Left / right / none hand
                'score'   : 0,                # Probability of predicted handedness (always>0.5, and opposite handedness=1-score)
                'angle'   : np.zeros(15),     # Flexion joint angles in degree
                'gesture' : None,             # Type of hand gesture
                'rvec'    : np.zeros(3),      # Global rotation vector Note: this term is only used for solvepnp initialization
                'tvec'    : np.asarray([0,0,0.6]), # Global translation vector (m) Note: Init z direc to some +ve dist (i.e. in front of camera), to prevent solvepnp from wrongly estimating z as -ve
                'fps'     : -1, # Frame per sec
                # https://github.com/google/mediapipe/issues/1351
                # 'visible' : np.zeros(21), # Visibility: Likelihood [0,1] of being visible (present and not occluded) in the image
                # 'presence': np.zeros(21), # Presence: Likelihood [0,1] of being present in the image or if its located outside the image
                'transformation': np.eye(4, dtype=np.float64),
                'static_transformation': np.eye(4, dtype=np.float64)
            }
            self.param.append(p)


    def result_to_param(self, result, img, static_hand = False):
        self.param[0]['count'] += 1 
        self.param[1]['count'] += 1
        # Convert mediapipe result to my own param
        img_height, img_width, _ = img.shape

        # Reset param
        for p in self.param:
            p['class'] = None

        if result.multi_hand_landmarks is not None:
            # Loop through different hands
            for i, res in enumerate(result.multi_handedness):
                if i>self.max_num_hands-1: break # Note: Need to check if exceed max number of hand
                self.param[i]['class'] = res.classification[0].label
                self.param[i]['score'] = res.classification[0].score

            # Loop through different hands
            for i, res in enumerate(result.multi_hand_landmarks):
                if i>self.max_num_hands-1: break # Note: Need to check if exceed max number of hand
                # Loop through 21 landmark for each hand
                for j, lm in enumerate(res.landmark):
                    self.param[i]['keypt'][j,0] = lm.x * img_width  # Convert normalized coor to pixel [0,1] -> [0,width]
                    self.param[i]['keypt'][j,1] = lm.y * img_height # Convert normalized coor to pixel [0,1] -> [0,height]
                    
                    # Raw normalized 2.5d keypoints in image coordinate
                    self.param[i]['keypt_raw'][j,0] = lm.x
                    self.param[i]['keypt_raw'][j,1] = lm.y
                    self.param[i]['keypt_raw'][j,2] = lm.z

                    # # Ignore it https://github.com/google/mediapipe/issues/1320
                    # self.param[i]['visible'][j] = lm.visibility
                    # self.param[i]['presence'][j] = lm.presence

        if result.multi_hand_world_landmarks is not None:
            for i, res in enumerate(result.multi_hand_world_landmarks):
                if i>self.max_num_hands-1: break # Note: Need to check if exceed max number of hand
                # Loop through 21 landmark for each hand
                for j, lm in enumerate(res.landmark):
                    self.param[i]['handworld_joint'][j,0] = lm.x
                    self.param[i]['handworld_joint'][j,1] = lm.y
                    self.param[i]['handworld_joint'][j,2] = lm.z

                # Convert relative 3D joint to angle
                self.param[i]['angle'] = self.convert_joint_to_angle(self.param[i]['handworld_joint'])
                # Convert relative 3D joint to camera coordinate
                self.convert_joint_to_camera_coor(self.param[i], self.intrin,use_solvepnp=True,static_hand=static_hand)

        return self.param

    
    def convert_joint_to_angle(self, joint):
        # Get direction vector of bone from parent to child
        v1 = joint[[0,1,2,3,0,5,6,7,0,9,10,11,0,13,14,15,0,17,18,19],:] # Parent joint
        v2 = joint[[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20],:] # Child joint
        v = v2 - v1 # [20,3]
        # Normalize v
        v = v/np.linalg.norm(v, axis=1)[:, np.newaxis]

        # Get angle using arcos of dot product
        angle = np.arccos(np.einsum('nt,nt->n',
            v[[0,1,2,4,5,6,8,9,10,12,13,14,16,17,18],:], 
            v[[1,2,3,5,6,7,9,10,11,13,14,15,17,18,19],:])) # [15,]

        return np.degrees(angle) # Convert radian to degree


    def convert_joint_to_camera_coor(self, param, intrin, use_solvepnp=True, static_hand = False):
        # MediaPipe version 0.8.9.1 onwards:
        # Given real-world 3D joint centered at middle MCP joint -> J_origin
        # To estimate the 3D joint in camera coordinate          -> J_camera = J_origin + tvec,
        # We need to find the unknown translation vector         -> tvec = [tx,ty,tz]
        # Such that when J_camera is projected to the 2D image plane
        # It matches the 2D keypoint locations

        # Considering all 21 keypoints,
        # Each keypoints will form 2 eq, in total we have 42 eq 3 unknowns
        # Since the equations are linear wrt [tx,ty,tz]
        # We can solve the unknowns using linear algebra A.x = b, where x = [tx,ty,tz]

        # Consider a single keypoint (pixel x) and joint (X,Y,Z)
        # Using the perspective projection eq:
        # (x - cx)/fx = (X + tx) / (Z + tz)
        # Similarly for pixel y:
        # (y - cy)/fy = (Y + ty) / (Z + tz)
        # Rearranging the above linear equations by keeping constants to the right hand side:
        # fx.tx - (x - cx).tz = -fx.X + (x - cx).Z
        # fy.ty - (y - cy).tz = -fy.Y + (y - cy).Z
        # Therefore, we can factor out the unknowns and form a matrix eq:
        # [fx  0 (x - cx)][tx]   [-fx.X + (x - cx).Z]
        # [ 0 fy (y - cy)][ty] = [-fy.Y + (y - cy).Z]
        #                 [tz]

        idx = [i for i in range(21)] # Use all landmarks

        if use_solvepnp:
            # Method 1: OpenCV solvePnP
            fx, fy = intrin['fx'], intrin['fy']
            cx, cy = intrin['cx'], intrin['cy']
            intrin_mat = np.asarray([[fx,0,cx],[0,fy,cy],[0,0,1]])
            dist_coeff = np.zeros(4)

            ret, param['rvec'], param['tvec'] = cv2.solvePnP(
                param['handworld_joint'][idx], param['keypt'][idx],
                intrin_mat, dist_coeff, param['rvec'], param['tvec'],
                useExtrinsicGuess=True)
            # Add tvec to all joints
            # param['joint'] += param['tvec']        
            
            R, _ = cv2.Rodrigues(param['rvec'])
            param['transformation'][:3,:3] = R
            param['transformation'][:3,3] = param['tvec'].squeeze()
            model_points_hom = np.concatenate((param['handworld_joint'], np.ones((21, 1))), axis=1)
            if param['count'] == 0:
                param['static_transformation'][:3,:3] = R
                param['static_transformation'][:3,3] = param['tvec'].squeeze()
            if not static_hand:
                world_points_in_camera = (model_points_hom @ param['transformation'].T)[:,0:3]
                # world_points_in_camera_z = world_points_in_camera[:, 2].reshape(-1, 1)
                # world_points_in_camera[:, :2] =  (np.concatenate([param['keypt'][idx], np.ones((param['keypt'][idx].shape[0], 1))], axis=1) @ np.linalg.inv(intrin_mat).T * world_points_in_camera_z)[:, :2]
                param['joint'] = world_points_in_camera
            else:
                param['joint'] = (model_points_hom @ param['static_transformation'].T)[:,0:3]
        else:
            # Method 2:
            A = np.zeros((len(idx),2,3))
            b = np.zeros((len(idx),2))

            A[:,0,0] = intrin['fx']
            A[:,1,1] = intrin['fy']
            A[:,0,2] = -(param['keypt'][idx,0] - intrin['cx'])
            A[:,1,2] = -(param['keypt'][idx,1] - intrin['cy'])

            b[:,0] = -intrin['fx'] * param['joint'][idx,0] \
                     + (param['keypt'][idx,0] - intrin['cx']) * param['handworld_joint'][idx,2]
            b[:,1] = -intrin['fy'] * param['joint'][idx,1] \
                     + (param['keypt'][idx,1] - intrin['cy']) * param['handworld_joint'][idx,2]

            A = A.reshape(-1,3) # [8,3]
            b = b.flatten() # [8]

            # Use the normal equation AT.A.x = AT.b to minimize the sum of the sq diff btw left and right sides
            x = np.linalg.solve(A.T @ A, A.T @ b)
            # Add tvec to all joints
            param['joint'] += x



    def forward(self, img, static_hand = False):
        # Preprocess image
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Extract result
        result = self.pipe.process(img)

        # Convert result to my own param
        param = self.result_to_param(result, img, static_hand)

        return param