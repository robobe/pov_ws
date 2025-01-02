from tf_transformations import (
    euler_from_quaternion,
    quaternion_from_euler,
    rotation_matrix,
    quaternion_from_matrix,
    quaternion_conjugate,
    quaternion_multiply
    
)

# import tf_transformations
# tf_transformations.quaternion_from_matrix
from math import pi
import numpy as np


y = rotation_matrix(pi/2,[0,0,1])
r = rotation_matrix(pi,[1,0,0])

enu_2_ned = y@r


enu = np.array([1.0 , 0.0, 0.0, 1.0])

ned = enu_2_ned@enu.T

print(f"enu: {np.round(enu, 1)}")
print(f"ned: {np.round(ned, 1)}")

print("---")

ned_2_enu = y@r
q = quaternion_from_matrix(ned_2_enu)
qc = quaternion_conjugate(q)

ned = quaternion_multiply(quaternion_multiply(q,enu), qc)[:3]

print(f"ned: {np.round(ned, 1)}")