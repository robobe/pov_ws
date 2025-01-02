import cv2
import cv2.aruco as aruco
import numpy as np
import matplotlib.pyplot as plt
# Step 1: Generate an ArUco marker
def generate_marker(dictionary_id, marker_id, side_pixels, output_file):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    border_size = 50 
    # Generate a marker
    marker_id = 42
    marker_size = 200  # Size in pixels
    marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
    image_with_border = cv2.copyMakeBorder(
        marker_image, 
        top=border_size, 
        bottom=border_size, 
        left=border_size, 
        right=border_size, 
        borderType=cv2.BORDER_CONSTANT, 
        value=(255, 255, 255)  # White color in BGR
    )
    cv2.imwrite('marker_42.png', image_with_border)
    plt.imshow(marker_image, cmap='gray', interpolation='nearest')
    plt.axis('off')  # Hide axes
    plt.title(f'ArUco Marker {marker_id}')
    plt.show()

# Generate a marker
def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []
    
    for c in corners:
        nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return rvecs, tvecs, trash
# Step 2: Detect the ArUco marker in an image
def detect_marker():

# Load the image
    image = cv2.imread('marker_42.png')
    matrix_coefficients = np.array(((933.15867, 0, 657.59), (0, 933.1586, 400.36993), (0, 0, 1)))
    distortion_coefficients = np.array((-0.43948, 0.18514, 0, 0))
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters()

    # Create the ArUco detector
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    # Detect the markers
    corners, ids, rejected = detector.detectMarkers(gray)
    # Print the detected markers
    print("Detected markers:", ids)
    if ids is not None:
        i = 0
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        rvec, tvec, trash = my_estimatePoseSingleMarkers(corners[i], 5.3, matrix_coefficients, distortion_coefficients)
        # rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.025, matrix_coefficients, distortion_coefficients)
        cv2.drawFrameAxes(image, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01) 
        cv2.imshow('Detected Markers', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

# Detect the marker in the generated image
generate_marker(aruco.DICT_6X6_250, 1, 300, "/tmp/marker_0.png")
detect_marker()
# detect_marker("/tmp/marker_0.png", aruco.DICT_6X6_250)

# Predefined camera matrix for 1280x720 resolution
camera_matrix = np.array([
    [800, 0, 640],  # Focal lengths (fx, fy) and principal point (cx)
    [0, 800, 360],
    [0, 0, 1]
], dtype=float)

# Predefined distortion coefficients (assuming minor distortion)
dist_coeffs = np.array([0, 0, 0, 0, 0], dtype=float)

print("Camera Matrix:\n", camera_matrix)
print("Distortion Coefficients:\n", dist_coeffs)
