# opencv-python==4.5.4.60
# opencv-contrib-python==4.5.4.60
import numpy as np
import cv2

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

id = 1
frame = np.zeros((300, 300, 1), dtype="uint8")
cv2.aruco.drawMarker(arucoDict, id, 300, frame, 1)
border_size = 10
image = cv2.copyMakeBorder(
        frame, 
        top=border_size, 
        bottom=border_size, 
        left=border_size, 
        right=border_size, 
        borderType=cv2.BORDER_CONSTANT, 
        value=(255, 255, 255)  # White color in BGR
    )
# cv2.imshow("arcuo", image)
cv2.imwrite('marker_1_4_4_50.png', image)
print(image.shape)
key = cv2.waitKey(0)
cv2.destroyAllWindows()


