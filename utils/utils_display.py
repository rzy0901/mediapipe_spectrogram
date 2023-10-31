###############################################################################
### adapted from https://github.com/ntu-rris/google-mediapipe
###############################################################################

import cv2
import numpy as np
import open3d as o3d


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


class DisplayHand:
    def __init__(self, draw3d=False, draw_camera=False, intrin=None, max_num_hands=1, vis=None):
        self.max_num_hands = max_num_hands
        if intrin is None:
            self.intrin = intrin_default
        else:
            self.intrin = intrin

        # Define kinematic tree linking keypoint together to form skeleton
        self.ktree = [0,          # Wrist
                      0,1,2,3,    # Thumb
                      0,5,6,7,    # Index
                      0,9,10,11,  # Middle
                      0,13,14,15, # Ring
                      0,17,18,19] # Little

        # Define color for 21 keypoint
        self.color = [[0,0,0], # Wrist black
                      [255,0,0],[255,60,0],[255,120,0],[255,180,0], # Thumb
                      [0,255,0],[60,255,0],[120,255,0],[180,255,0], # Index
                      [0,255,0],[0,255,60],[0,255,120],[0,255,180], # Middle
                      [0,0,255],[0,60,255],[0,120,255],[0,180,255], # Ring
                      [0,0,255],[60,0,255],[120,0,255],[180,0,255]] # Little
        self.color = np.asarray(self.color)
        self.color_ = self.color / 255 # For Open3D RGB
        self.color[:,[0,2]] = self.color[:,[2,0]] # For OpenCV BGR
        self.color = self.color.tolist()


        ############################
        ### Open3D visualization ###
        ############################
        if draw3d:
            if vis is not None:
                self.vis = vis
            else:
                self.vis = o3d.visualization.Visualizer()
                self.vis.create_window(
                    width=self.intrin['width'], height=self.intrin['height'],window_name='Motion Converter')
            self.vis.get_render_option().point_size = 8.0
            joint = np.zeros((21,3))

            # Draw 21 joints
            self.pcd = []
            for i in range(max_num_hands):
                p = o3d.geometry.PointCloud()
                p.points = o3d.utility.Vector3dVector(joint)
                p.colors = o3d.utility.Vector3dVector(self.color_)
                self.pcd.append(p)
            
            # Draw 20 bones
            self.bone = []
            for i in range(max_num_hands):
                b = o3d.geometry.LineSet()
                b.points = o3d.utility.Vector3dVector(joint)
                b.colors = o3d.utility.Vector3dVector(self.color_[1:])
                b.lines  = o3d.utility.Vector2iVector(
                    [[0,1], [1,2],  [2,3],  [3,4],    
                     [0,5], [5,6],  [6,7],  [7,8],   
                     [9,10], [10,11],[11,12],  
                     [13,14],[14,15],[15,16],  
                     [0,17],[17,18],[18,19],[19,20],
                     [5,9],[9,13],[13,17]]) 
                self.bone.append(b)

            # Draw world reference frame
            frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)

            # Add geometry to visualize
            self.vis.add_geometry(frame)
            for i in range(max_num_hands):
                self.vis.add_geometry(self.pcd[i])
                self.vis.add_geometry(self.bone[i])

            # Set camera view
            ctr = self.vis.get_view_control()
            ctr.set_up([0,-1,0]) # Set up as -y axis
            ctr.set_front([0,0,-1]) # Set to looking towards -z axis
            ctr.set_lookat([0,0,0]) # Set to center of view
            ctr.set_zoom(1)
            
            if draw_camera:
                # Remove previous frame
                self.vis.remove_geometry(frame)
                # Draw camera reference frame
                frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
                # Draw camera frustum
                self.camera = DisplayCamera(self.vis, self.intrin)
                frustum = self.camera.create_camera_frustum()
                # Draw 2D image plane in 3D space
                self.mesh_img = self.camera.create_mesh_img()
                # Add geometry to visualize
                self.vis.add_geometry(frame)
                self.vis.add_geometry(frustum)
                self.vis.add_geometry(self.mesh_img)
                # Reset camera view
                self.camera.reset_view()

    # Default 2d plot in mediapipe
    def draw2d_default(self,img,param):
        from mediapipe.python.solutions.hands_connections import HAND_CONNECTIONS
        from mediapipe import solutions
        from mediapipe.framework.formats import landmark_pb2
        img_height, img_width, _ = img.shape
        for p in param:
            if p['class'] is not None:
                hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
                hand_landmarks_proto.landmark.extend([landmark_pb2.NormalizedLandmark(x=landmark[0], y=landmark[1], z=landmark[2]) for landmark in p['keypt_raw']])
                solutions.drawing_utils.draw_landmarks(
                    img,
                    hand_landmarks_proto,
                    solutions.hands.HAND_CONNECTIONS,
                    solutions.drawing_styles.get_default_hand_landmarks_style(),
                    solutions.drawing_styles.get_default_hand_connections_style())
            # Label fps
            if p['fps']>0:
                cv2.putText(img, 'FPS: %.1f' % (p['fps']),
                    (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)   


    def draw2d(self, img, param):
        img_height, img_width, _ = img.shape

        # Loop through different hands
        for p in param:
            if p['class'] is not None:
                # Label left or right hand
                # x = int(p['keypt'][0,0]) - 30
                # y = int(p['keypt'][0,1]) + 40
                # cv2.putText(img, '%s %.3f' % (p['class'], p['score']), (x, y), 
                # cv2.putText(img, '%s' % (p['class']), (x, y), 
                #     cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2) # Red
                
                # Loop through keypoint for each hand
                for i in range(21):
                    x = int(p['keypt'][i,0])
                    y = int(p['keypt'][i,1])
                    if x>0 and y>0 and x<img_width and y<img_height:
                        # Draw skeleton
                        start = p['keypt'][self.ktree[i],:]
                        x_ = int(start[0])
                        y_ = int(start[1])
                        if x_>0 and y_>0 and x_<img_width and y_<img_height:
                            cv2.line(img, (x_, y_), (x, y), self.color[i], 2) 

                        # Draw keypoint
                        cv2.circle(img, (x, y), 5, self.color[i], -1)
                        # cv2.circle(img, (x, y), 3, self.color[i], -1)

                        # # Number keypoint
                        # cv2.putText(img, '%d' % (i), (x, y), 
                        #     cv2.FONT_HERSHEY_SIMPLEX, 1, self.color[i])

                        # # Label visibility and presence
                        # cv2.putText(img, '%.1f, %.1f' % (p['visible'][i], p['presence'][i]),
                        #     (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, self.color[i])
                
                # Label gesture
                if p['gesture'] is not None:
                    size = cv2.getTextSize(p['gesture'].upper(), 
                        # cv2.FONT_HERSHEY_SIMPLEX, 2, 2)[0]
                        cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
                    x = int((img_width-size[0]) / 2)
                    # cv2.putText(img, p['gesture'].upper(),
                    #     # (x, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255), 2)
                    #     (x, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

                    # Label joint angle
                    self.draw_joint_angle(img, p)

            # # Label fps
            # if p['fps']>0:
            #     cv2.putText(img, 'FPS: %.1f' % (p['fps']),
            #         (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)   

        return img


    def draw2d_(self, img, param):
        # Different from draw2d
        # draw2d_ draw 2.5D with relative depth info
        # The closer the landmark is towards the camera
        # The lighter and larger the circle

        img_height, img_width, _ = img.shape

        # Loop through different hands
        for p in param:
            if p['class'] is not None:
                # Extract wrist pixel
                x = int(p['keypt'][0,0]) - 30
                y = int(p['keypt'][0,1]) + 40
                # Label left or right hand
                # cv2.putText(img, '%s %.3f' % (p['class'], p['score']), (x, y), 
                #     cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2) # Red

                min_depth = min(p['joint'][:,2])
                max_depth = max(p['joint'][:,2])

                # Loop through keypt and joint for each hand
                for i in range(21):
                    x = int(p['keypt'][i,0])
                    y = int(p['keypt'][i,1])
                    if x>0 and y>0 and x<img_width and y<img_height:
                        # Convert depth to color nearer white, further black
                        depth = (max_depth-p['joint'][i,2]) / (max_depth-min_depth)
                        color = [int(255*depth), int(255*depth), int(255*depth)]
                        size = int(10*depth)+2
                        # size = 2

                        # Draw skeleton
                        start = p['keypt'][self.ktree[i],:]
                        x_ = int(start[0])
                        y_ = int(start[1])
                        if x_>0 and y_>0 and x_<img_width and y_<img_height:
                            cv2.line(img, (x_, y_), (x, y), color, 2)

                        # Draw keypoint
                        cv2.circle(img, (x, y), size, color, size)
            
            # Label fps
            # if p['fps']>0:
            #     cv2.putText(img, 'FPS: %.1f' % (p['fps']),
            #         (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)                           

        return img


    def draw3d(self, param, img=None):
        for i in range(self.max_num_hands):
            if param[i]['class'] is None:
                self.pcd[i].points = o3d.utility.Vector3dVector(np.zeros((21,3)))
                self.bone[i].points = o3d.utility.Vector3dVector(np.zeros((21,3)))
            else:
                self.pcd[i].points = o3d.utility.Vector3dVector(param[i]['joint'])
                self.bone[i].points = o3d.utility.Vector3dVector(param[i]['joint'])

        if img is not None:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            self.mesh_img.textures = [o3d.geometry.Image(img)]            


    def draw_joint_angle(self, img, p):
        # Create text
        text = None
        if p['gesture']=='Finger MCP Flexion':
            text = 'Index : %.1f \
                  \nMiddle: %.1f \
                  \nRing  : %.1f \
                  \nLittle : %.1f' % \
                (p['angle'][3], p['angle'][6], p['angle'][9], p['angle'][12])

        elif p['gesture']=='Finger PIP DIP Flexion':
            text = 'PIP: \
                  \nIndex : %.1f \
                  \nMiddle: %.1f \
                  \nRing  : %.1f \
                  \nLittle : %.1f \
                  \nDIP: \
                  \nIndex : %.1f \
                  \nMiddle: %.1f \
                  \nRing  : %.1f \
                  \nLittle : %.1f' % \
                (p['angle'][4], p['angle'][7], p['angle'][10], p['angle'][13],
                 p['angle'][5], p['angle'][8], p['angle'][11], p['angle'][14])

        elif p['gesture']=='Thumb MCP Flexion':
            text = 'Angle: %.1f' % p['angle'][1]

        elif p['gesture']=='Thumb IP Flexion':
            text = 'Angle: %.1f' % p['angle'][2]

        elif p['gesture']=='Thumb Radial Abduction':
            text = 'Angle: %.1f' % p['angle'][0]

        elif p['gesture']=='Thumb Palmar Abduction':
            text = 'Angle: %.1f' % p['angle'][0]

        elif p['gesture']=='Thumb Opposition':
            # Dist btw thumb and little fingertip
            dist = np.linalg.norm(p['joint'][4] - p['joint'][20])
            text = 'Dist: %.3f' % dist
        
        elif p['gesture']=='Forearm Neutral' or \
             p['gesture']=='Forearm Pronation' or \
             p['gesture']=='Forearm Supination' or \
             p['gesture']=='Wrist Flex/Extension' or \
             p['gesture']=='Wrist Radial/Ulnar Dev':
            text = 'Angle: %.1f' % p['angle'][0]

        if text is not None:
            x0 = 10 # Starting x coor for placing text
            y0 = 60 # Starting y coor for placing text
            dy = 25 # Change in text vertical spacing        
            vert = len(text.split('\n'))
            # Draw black background
            cv2.rectangle(img, (x0, y0), (140, y0+vert*dy+10), (0,0,0), -1)
            # Draw text
            for i, line in enumerate(text.split('\n')):
                y = y0 + (i+1)*dy
                cv2.putText(img, line,
                    (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)


    def draw_game_rps(self, img, param):
        img_height, img_width, _ = img.shape

        # Init result of 2 hands to none
        res = [None, None]

        # Loop through different hands
        for j, p in enumerate(param):
            # Only allow maximum of two hands
            if j>1:
                break

            if p['class'] is not None:                
                # Loop through keypoint for each hand
                for i in range(21):
                    x = int(p['keypt'][i,0])
                    y = int(p['keypt'][i,1])
                    if x>0 and y>0 and x<img_width and y<img_height:
                        # Draw skeleton
                        start = p['keypt'][self.ktree[i],:]
                        x_ = int(start[0])
                        y_ = int(start[1])
                        if x_>0 and y_>0 and x_<img_width and y_<img_height:
                            cv2.line(img, (x_, y_), (x, y), self.color[i], 2)

                        # Draw keypoint
                        cv2.circle(img, (x, y), 5, self.color[i], -1)

                # Label gesture 
                text = None
                if p['gesture']=='fist':
                    text = 'rock'
                elif p['gesture']=='five':
                    text = 'paper'
                elif (p['gesture']=='three') or (p['gesture']=='yeah'):
                    text = 'scissor'
                res[j] = text

                # Label result
                if text is not None:
                    x = int(p['keypt'][0,0]) - 30
                    y = int(p['keypt'][0,1]) + 40
                    cv2.putText(img, '%s' % (text.upper()), (x, y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2) # Red

        # Determine winner
        text = None
        winner = None
        if res[0]=='rock':
            if res[1]=='rock'     : text = 'Tie'
            elif res[1]=='paper'  : text = 'Paper wins'  ; winner = 1
            elif res[1]=='scissor': text = 'Rock wins'   ; winner = 0
        elif res[0]=='paper':
            if res[1]=='rock'     : text = 'Paper wins'  ; winner = 0
            elif res[1]=='paper'  : text = 'Tie'
            elif res[1]=='scissor': text = 'Scissor wins'; winner = 1
        elif res[0]=='scissor':
            if res[1]=='rock'     : text = 'Rock wins'   ; winner = 1
            elif res[1]=='paper'  : text = 'Scissor wins'; winner = 0
            elif res[1]=='scissor': text = 'Tie'

        # Label gesture
        if text is not None:
            size = cv2.getTextSize(text.upper(), 
                # cv2.FONT_HERSHEY_SIMPLEX, 2, 2)[0]
                cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
            x = int((img_width-size[0]) / 2)
            cv2.putText(img, text.upper(),
                # (x, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255), 2)
                (x, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)            

        # Draw winner text
        if winner is not None:
            x = int(param[winner]['keypt'][0,0]) - 30
            y = int(param[winner]['keypt'][0,1]) + 80
            cv2.putText(img, 'WINNER', (x, y), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2) # Yellow            

        return img

class DisplayCamera:
    def __init__(self, vis, intrin=None):
        self.vis = vis
        
        # Get camera intrinsics param
        if intrin is None:
            self.intrin = intrin_default
        else:
            self.intrin = intrin            
        
        # Note: Need to subtract optical center by 0.5
        # https://github.com/intel-isl/Open3D/issues/727
        self.intrin['cx'] -= 0.5
        self.intrin['cy'] -= 0.5

        # For reset_view
        self.pinhole = o3d.camera.PinholeCameraParameters()
        self.pinhole.extrinsic = np.eye(4)
        self.pinhole.intrinsic = o3d.camera.PinholeCameraIntrinsic(
            self.intrin['width'], self.intrin['height'], 
            self.intrin['fx'], self.intrin['fy'], 
            self.intrin['cx'], self.intrin['cy'])


    def reset_view(self):
        # Reset camera view to this camera
        self.vis.get_view_control().convert_from_pinhole_camera_parameters(
            self.pinhole)


    def unproject_pt(self, u, v, depth):
        # Transform 2D pixels to 3D points
        # Given pixel coordinates and depth in an image
        # with no distortion or inverse distortion coefficients
        # compute the corresponding point in 3D space relative to the same camera
        x = (u - self.intrin['cx'])/self.intrin['fx']*depth
        y = (v - self.intrin['cy'])/self.intrin['fy']*depth
        z = depth

        return [x, y, z]


    def create_camera_frustum(self, depth=[0.5,1.0]):
        # Get camera intrinsics param
        w  = self.intrin['width']
        h  = self.intrin['height']

        # Each frustum 8 lines, each line 2 pts, and plot at 2 different depths
        points, lines = [], []
        c = 0 # Counter
        points.append([0,0,0]) # Origin
        for d in depth: # Plot at different depth
            points.append(self.unproject_pt(0, 0, d)) # Top left
            points.append(self.unproject_pt(w, 0, d)) # Top right
            points.append(self.unproject_pt(w, h, d)) # Bottom left
            points.append(self.unproject_pt(0, h, d)) # Bottom right
            lines.append([0,c+1]);   lines.append([0,c+2])
            lines.append([0,c+3]);   lines.append([0,c+4])
            lines.append([c+1,c+2]); lines.append([c+2,c+3])
            lines.append([c+3,c+4]); lines.append([c+4,c+1])
            c += 4
            
        # Set to uniform light gray color
        colors = [[0.75,0.75,0.75] for i in range(len(lines))]

        line = o3d.geometry.LineSet()
        line.lines  = o3d.utility.Vector2iVector(lines)
        line.points = o3d.utility.Vector3dVector(points)
        line.colors = o3d.utility.Vector3dVector(colors)

        return line


    def create_mesh_img(self, img=None, depth=1.0):
        # Get camera intrinsics param
        w  = self.intrin['width']
        h  = self.intrin['height']

        if img is None:
            img = np.zeros((h, w, 3), dtype=np.uint8)

        vert = []
        vert.append(self.unproject_pt( 0, 0, depth)) # Top left
        vert.append(self.unproject_pt( w, 0, depth)) # Top right
        vert.append(self.unproject_pt( w, h, depth)) # Bottom left
        vert.append(self.unproject_pt( 0, h, depth)) # Bottom right

        mesh = o3d.geometry.TriangleMesh()
        # Convention of 4 vertices
        # --> right x
        # Down y
        # Vertex 0 (-x,-y) -- Vertex 1 (x,-y)
        # Vertex 3 (-x,y)  -- Vertex 2 (x,y)
        mesh.vertices = o3d.utility.Vector3dVector(vert)
        # Anti-clockwise direction (4 triangles to allow two-sided views)
        mesh.triangles = o3d.utility.Vector3iVector(
            [[0,2,1],[0,3,2],  # Front face
             [0,1,2],[0,2,3]]) # Back face
        # Define the uvs to match img coor to the order of triangles
        # Top left    (0,0) -- Top right    (1,0)
        # Bottom left (0,1) -- Bottom right (1,1)
        mesh.triangle_uvs = o3d.utility.Vector2dVector(
            [[0,0],[1,1],[1,0], [0,0],[0,1],[1,1], # Front face
             [0,0],[1,0],[1,1], [0,0],[1,1],[0,1]]) # Back face
        # Image to be displayed
        mesh.textures = [o3d.geometry.Image(img)]

        num_face = np.asarray(mesh.triangles).shape[0]
        mesh.triangle_material_ids = o3d.utility.IntVector(
            np.zeros(num_face, dtype=np.int32))

        return mesh             