# maps stationary bike IMU coordinates to new x,y,z coordinate system
# prints out a corresponding rotation matrix
#
# When finding the bike stationary coords, ensure you plumb the frame
# instead of trying to balance it because bikes are uneven
#
# The way I mounted it, y is longitudinal and x is transverse horizontal
# z is vertical
# do this with a rotation matrix

import numpy as np

# offsets

#previously from nonfoam frame-mounted bno: (x,y,z) = (-0.36, 0.82, 9.67)
x_stationary = 1.23

y_stationary = 3.18

z_stationary = -9.11

g_stationary = [x_stationary, y_stationary, z_stationary]
mag = np.sqrt(x_stationary**2 + y_stationary**2 + z_stationary**2)

znew = g_stationary/mag # our desired z vector in current reference frame
# to clarify, we want a plumb, level, stationary bike to have ONLY z acceleration.

# start with x because when riding, it's more independent from z than y is from z
x_ideal = np.array([1,0,0])
# find correlation to z desired and subtract from ideal x
xnew = x_ideal-np.dot(x_ideal, znew)*znew 
xnew = xnew/np.linalg.norm(xnew) # normalize

ynew = np.cross(znew, xnew)
# znew and xnew are orthogonal unit vectors, so x product is unit length

# rotation matrix:
R = np.column_stack([xnew, ynew, znew])

# must invert this to map sensor readings to desired dimensions
# since it is orthogonal, R inv = R transpose

R = R.T
print(R)
