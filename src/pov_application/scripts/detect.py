# work with opencv 4.6.6
import cv2 # Import the OpenCV library
import numpy as np # Import Numpy library
import pathlib 
# Project: ArUco Marker Detector
# Date created: 12/18/2021
# Python version: 3.8
# Reference: https://www.pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
#  https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
desired_aruco_dictionary = "DICT_ARUCO_ORIGINAL"
 
# The different ArUco dictionaries built into the OpenCV library. 
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

def main():

    # this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT["DICT_7X7_50"])
    this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT["DICT_7X7_50"])
    
    size = 200
    border_size = 50  # Example: 50 pixels on each side

# Add a white border around the image
    
    frame = np.zeros((size, size, 1), dtype="uint8")
    cv2.aruco.drawMarkerAruco(this_aruco_dictionary, 5, size, frame, 1)

    image_with_border = cv2.copyMakeBorder(
        frame, 
        top=border_size, 
        bottom=border_size, 
        left=border_size, 
        right=border_size, 
        borderType=cv2.BORDER_CONSTANT, 
        value=(255, 255, 255)  # White color in BGR
    )

    

    this_aruco_parameters = cv2.aruco.DetectorParameters_create()
    
    # frame = cv2.imread("DICT_ARUCO_ORIGINAL_id1.png")
    # Detect ArUco markers in the video frame
    (corners, ids, rejected) = cv2.aruco.detectMarkers(
        image_with_border, this_aruco_dictionary, parameters=this_aruco_parameters)
    
    # Check that at least one ArUco marker was detected
    if len(corners) > 0:
        # Flatten the ArUco IDs list
        ids = ids.flatten()
        print("---")
        print(ids)
        print(corners)
        # Loop over the detected ArUco corners
        # for (marker_corner, marker_id) in zip(corners, ids):
        
        # # Extract the marker corners
        # corners = marker_corner.reshape((4, 2))
        # (top_left, top_right, bottom_right, bottom_left) = corners
            
        # # Convert the (x,y) coordinate pairs to integers
        # top_right = (int(top_right[0]), int(top_right[1]))
        # bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
        # bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
        # top_left = (int(top_left[0]), int(top_left[1]))
            
        # # Draw the bounding box of the ArUco detection
        # cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
        # cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
        # cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
        # cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)
            
        # # Calculate and draw the center of the ArUco marker
        # center_x = int((top_left[0] + bottom_right[0]) / 2.0)
        # center_y = int((top_left[1] + bottom_right[1]) / 2.0)
        # cv2.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)
            
        # # Draw the ArUco marker ID on the video frame
        # # The ID is always located at the top_left of the ArUco marker
        # cv2.putText(frame, str(marker_id), 
        #     (top_left[0], top_left[1] - 15),
        #     cv2.FONT_HERSHEY_SIMPLEX,
        #     0.5, (0, 255, 0), 2)

# Display the resulting frame
    # "/workspaces/pov_ws/src/pov_description/models/arcuo/materials/textures"
    output_file = pathlib.Path("").joinpath("5_7_7_50.png").as_posix()
    cv2.imshow('frame',image_with_border)
    # cv2.imwrite(output_file, image_with_border)
        
# If "q" is pressed on the keyboard, 
# exit this loop
    cv2.waitKey(5000) 
    
    # Close down the video stream
    cv2.destroyAllWindows()
   
if __name__ == '__main__':
  print(cv2.__version__)
  main()