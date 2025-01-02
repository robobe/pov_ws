import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Create a 3D figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot a point for demonstration
ax.scatter(1, 2, 3, color='red', label='Example Point')

# Set the range for each axis
ax.set_xlim([-10, 10])  # X-axis range
ax.set_ylim([-20, 20])  # Y-axis range
ax.set_zlim([-5, 5])    # Z-axis range

# Add axis labels
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')

# Add a legend
ax.legend()

# Show the plot
plt.show()