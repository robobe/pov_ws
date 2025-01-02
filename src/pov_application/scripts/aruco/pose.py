# https://stackoverflow.com/questions/46363618/aruco-markers-with-opencv-get-the-3d-corner-coordinates?rq=1


# opencv-python==4.5.4.60
# opencv-contrib-python==4.5.4.60

import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import math # Math library
# from transforms3d
from tf_transformations import euler_from_quaternion

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

id = 1
frame = np.zeros((300, 300, 1), dtype="uint8")
cv2.aruco.drawMarker(arucoDict, id, 300, frame, 1)
border_size =10
image = cv2.copyMakeBorder(
        frame, 
        top=border_size, 
        bottom=border_size, 
        left=border_size, 
        right=border_size, 
        borderType=cv2.BORDER_CONSTANT, 
        value=(255, 255, 255)  # White color in BGR
    )

arucoParams = cv2.aruco.DetectorParameters_create()
(corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict,
	parameters=arucoParams)

print(ids)
print(corners)
image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

 # Draw a square around detected markers in the video frame
cv2.aruco.drawDetectedMarkers(image, corners, ids)

aruco_marker_side_length = 0.5
camera_matrix = np.array([
    [800, 0, 640],  # Focal lengths (fx, fy) and principal point (cx)
    [0, 800, 360],
    [0, 0, 1]
], dtype=float)

# Predefined distortion coefficients (assuming minor distortion)
dist_coeffs = np.array([0, 0, 0, 0, 0], dtype=float)

# Get the rotation and translation vectors
rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
corners,
aruco_marker_side_length,
camera_matrix,
dist_coeffs)

for i, marker_id in enumerate(ids):
       
    # Store the translation (i.e. position) information
    transform_translation_x = tvecs[i][0][0]
    transform_translation_y = tvecs[i][0][1]
    transform_translation_z = tvecs[i][0][2]

    # Store the rotation information
    rotation_matrix = np.eye(4)
    rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
    r = R.from_matrix(rotation_matrix[0:3, 0:3])
    quat = r.as_quat()   
        
    # Quaternion format     
    transform_rotation_x = quat[0] 
    transform_rotation_y = quat[1] 
    transform_rotation_z = quat[2] 
    transform_rotation_w = quat[3] 
        
    # Euler angle format in radians
    roll_x, pitch_y, yaw_z = euler_from_quaternion([transform_rotation_x, 
                                                    transform_rotation_y, 
                                                    transform_rotation_z, 
                                                    transform_rotation_w])
        
    roll_x = math.degrees(roll_x)
    pitch_y = math.degrees(pitch_y)
    yaw_z = math.degrees(yaw_z)
    print("transform_translation_x: {}".format(transform_translation_x))
    print("transform_translation_y: {}".format(transform_translation_y))
    print("transform_translation_z: {}".format(transform_translation_z))
    print("roll_x: {}".format(roll_x))
    print("pitch_y: {}".format(pitch_y))
    print("yaw_z: {}".format(yaw_z))
    print()
        
    # Draw the axes on the marker
    cv2.aruco.drawAxis(image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.05)
    
# Display the resulting frame
cv2.imshow('frame',image)
cv2.waitKey(0)