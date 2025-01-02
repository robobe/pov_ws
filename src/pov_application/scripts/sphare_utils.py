
import numpy as np

class ArcIterator:
    def __init__(self, center, radius, start_angle, end_angle, num_points, plane='xy'):
        self.__center = center
        self.__radius = radius
        self.__start = start_angle
        self.__end = end_angle
        self.__num_points = num_points
        self.__plane = plane
        self.__start_rad = np.radians(self.__start)
        self.__end_rad = np.radians(self.__end)
    
        # Generate angles along the arc
        self.__angles = np.linspace(self.__start_rad, self.__end_rad, num_points).tolist()
        self.__current = -1

    def __iter__(self):
        return self

    def __next__(self): # Python 2: def next(self)
        self.__current += 1
        if self.__current < len(self.__angles):
            angle = self.__angles[self.__current]
            if plane == 'xy':
                x = center[0] + radius * np.cos(angle)
                y = center[1] + radius * np.sin(angle)
                z = center[2]
            elif plane == 'xz':
                x = center[0] + radius * np.cos(angle)
                y = center[1]
                z = center[2] + radius * np.sin(angle)
            elif plane == 'yz':
                x = center[0]
                y = center[1] + radius * np.cos(angle)
                z = center[2] + radius * np.sin(angle)
            else:
                raise ValueError("Invalid plane. Choose from 'xy', 'xz', or 'yz'.")
            return(x, y, z)
        raise StopIteration




def calculate_arc_points(center, radius, start_angle, end_angle, num_points, plane='xy'):
    """
    Calculate points on a circular arc in 3D space.

    Parameters:
        center (tuple): The (x, y, z) coordinates of the circle's center.
        radius (float): The radius of the arc.
        start_angle (float): Start angle in degrees.
        end_angle (float): End angle in degrees.
        num_points (int): Number of points to calculate along the arc.
        plane (str): Plane of the arc ('xy', 'xz', 'yz').

    Returns:
        list: A list of (x, y, z) tuples representing points on the arc.
    """
    # Convert angles from degrees to radians
    start_rad = np.radians(start_angle)
    end_rad = np.radians(end_angle)
    
    # Generate angles along the arc
    angles = np.linspace(start_rad, end_rad, num_points)
    
    # Calculate points based on the selected plane
    points = []
    for angle in angles:
        if plane == 'xy':
            x = center[0] + radius * np.cos(angle)
            y = center[1] + radius * np.sin(angle)
            z = center[2]
        elif plane == 'xz':
            x = center[0] + radius * np.cos(angle)
            y = center[1]
            z = center[2] + radius * np.sin(angle)
        elif plane == 'yz':
            x = center[0]
            y = center[1] + radius * np.cos(angle)
            z = center[2] + radius * np.sin(angle)
        else:
            raise ValueError("Invalid plane. Choose from 'xy', 'xz', or 'yz'.")
        points.append((x, y, z))
    
    return points

# Example usage
center = (0, 0, 0)  # Circle center
radius = 5          # Radius of the circle
start_angle = 0     # Start angle in degrees
end_angle = 110      # End angle in degrees
num_points = 40     # Number of points on the arc
plane = 'xz'        # Arc plane

arc_points = ArcIterator(center, radius, start_angle, end_angle, num_points, plane)
for point in arc_points:
    print(point)
