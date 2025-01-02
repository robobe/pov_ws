import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def calculate_rpy(point1, point2):
    # Compute the vector from point2 to point1
    vector = np.array(point1) - np.array(point2)
    dx, dy, dz = vector
    dx = -dx
    dy - -dy
    # Calculate yaw (rotation around Z-axis)
    yaw = np.arctan2(dy, dx)  # Angle in radians

    # Calculate pitch (rotation around Y-axis)
    pitch = np.arctan2(-dz, np.sqrt(dx**2 + dy**2))  # Angle in radians

    # Roll is assumed to be 0 as it's not defined for a single vector
    roll = 0.0

    # Convert from radians to degrees if needed
    yaw_deg = np.degrees(yaw)
    pitch_deg = np.degrees(pitch)
    roll_deg = np.degrees(roll)

    return roll_deg, pitch_deg, yaw_deg

# Define two points in 3D space
point1 = np.array([0, 0, 0])
point2 = np.array([-4, 6, 8])

# Compute the vector from point1 to point2
vector = point1 - point2
r,p,y = calculate_rpy(point2, point1)
print(r,p,y)
limit = 5  # Adjust this based on the range of your data

# Plot the points and the vector
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the points
ax.scatter(*point1, color='blue', label='Point 1')
ax.scatter(*point2, color='red', label='Point 2')
ax.set_xlim([-limit, limit])
ax.set_ylim([-limit, limit])
ax.set_zlim([0, limit*2])
# Plot the vector
ax.quiver(*point2, *vector, color='green', label='Vector', arrow_length_ratio=0.1)

# Set labels and legend
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()


# Show the plot
plt.show()
