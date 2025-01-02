import numpy as np

def rotate_vector_by_yaw(vector, yaw):
    # Rotation matrix for yaw (around Z-axis in 3D)
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # Rotate the vector
    rotated_vector = np.dot(Rz, vector)
    
    return rotated_vector

# Example vector (in 3D)
vector = np.array([1.0, 0.0, 0.0])

# Yaw angle in radians (e.g., 30 degrees)
yaw = np.radians(-90)

# Rotate the vector
rotated_vector = rotate_vector_by_yaw(vector, yaw)

print("Original Vector:", vector)
print("Rotated Vector:", rotated_vector)
