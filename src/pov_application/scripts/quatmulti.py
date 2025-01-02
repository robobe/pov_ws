from tf_transformations import (
    quaternion_multiply,
    quaternion_from_euler,
    quaternion_conjugate,
    translation_matrix,
    rotation_matrix
)
import tf_transformations
import math
import numpy as np
from math import pi

# tf_transformations.rotation_matrix()
# vector = np.array([1, 0, 0])  # Example vector
# quat = quaternion_from_euler(0, 0, math.pi/2)

# q = quat / np.linalg.norm(quat)
# vector_as_quat = [vector[0], vector[1], vector[2], 0]
# q_conjugate = np.array([-q[0], -q[1], -q[2], q[3]])

# q = quaternion_multiply(quaternion_multiply(q,vector_as_quat), q_conjugate)
# v = np.array(q[:3])
# v= np.round(v, 1)
# print(v)

#input vector
vector = np.array([1, 0, 0, 1])  # Example vector
# create rotation
quat = quaternion_from_euler(0, 0, math.pi/2)
vector_as_quat = [vector[0], vector[1], vector[2], 0]
# conjugate quaternion
q_conjugate = quaternion_conjugate(quat)
#multiple
q = quaternion_multiply(quaternion_multiply(quat, vector_as_quat), q_conjugate)
# extract vector from result
v = np.array(q[:3])
# round for batter view
v= np.round(v, 1)
print(v)

y = rotation_matrix(pi/2, [0,0,1])
print(np.round(y,1))
r = rotation_matrix(pi/2, [1,0,0])
print(np.round(r,1))

m = np.dot(y,r)

v_tag = np.dot(m,vector)

print(v_tag)