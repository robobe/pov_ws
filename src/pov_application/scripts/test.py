import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def enu_to_camera_optical(point_enu):
    # ENU to Camera Optical rotation matrix (3D)
    rotation_matrix = np.array([
        [-1, 0,  0],
        [ 0, 0, -1],
        [ 0, 1,  0]
    ])
    
    # Transform the point
    point_camera = rotation_matrix @ np.array(point_enu)
    return point_camera

def plot_transformed_point():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Define the axes for the ENU frame
    ax.quiver(0, 0, 0, 1, 0, 0, color='r', label='East (X)')
    ax.quiver(0, 0, 0, 0, 1, 0, color='g', label='North (Y)')
    ax.quiver(0, 0, 0, 0, 0, 1, color='b', label='Up (Z)')

    # Mark a point in ENU and transformed point in Camera Optical
    point_enu = [0, 1, 0]
    xc = enu_to_camera_optical(point_enu)
    ax.quiver(0, 0, xc[0], xc[1], xc[2], 0, color='y', label='xc (X)')

    # Plot both points
    # ax.scatter(point_enu[0], point_enu[1], point_enu[2], color='purple', label=f"Point ENU: {point_enu}")
    # ax.scatter(point_camera[0], point_camera[1], point_camera[2], color='orange', label=f"Point Camera Optical: {point_camera}")

    # Set plot limits
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.legend()
    ax.grid(True)

    # Show the plot
    plt.title('Coordinate Transformation from ENU to Camera Optical')
    plt.show()

plot_transformed_point()
