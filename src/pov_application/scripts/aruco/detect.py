# opencv-python==4.5.4.60
# opencv-contrib-python==4.5.4.60

import numpy as np
import cv2

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
# loop over the detected ArUCo corners
for (markerCorner, markerID) in zip(corners, ids):
    # extract the marker corners (which are always returned in
    # top-left, top-right, bottom-right, and bottom-left order)
    corners = markerCorner.reshape((4, 2))
    (topLeft, topRight, bottomRight, bottomLeft) = corners

    # convert each of the (x, y)-coordinate pairs to integers
    topRight = (int(topRight[0]), int(topRight[1]))
    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
    topLeft = (int(topLeft[0]), int(topLeft[1]))

    # draw the bounding box of the ArUCo detection
    cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
    cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
    cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
    cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

    # compute and draw the center (x, y)-coordinates of the ArUco
    # marker
    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
    cY = int((topLeft[1] + bottomRight[1]) / 2.0)
    cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

    # draw the ArUco marker ID on the image
    cv2.putText(image, str(markerID),
        (topLeft[0], topLeft[1] + 15), cv2.FONT_HERSHEY_SIMPLEX,
        0.5, (0, 255, 0), 2)
    print("[INFO] ArUco marker ID: {}".format(markerID))

    # show the output image
    cv2.imshow("aaa", image)
    cv2.waitKey(0)