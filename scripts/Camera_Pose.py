#!/usr/bin/env python2
import numpy as np, tf
from IPython import embed
x = 2
# y = -0.4
y = 0.
z = 0.8

baxter_actual_height = 0.25

theta = np.math.atan2(y,x)
phi = np.math.atan2(z-baxter_actual_height,np.sqrt(x**2+y**2))

#ORDER ZXY
rotate_z = theta+np.pi/2
rotate_x = -(phi+np.pi/2)

print("Z:", rotate_z)
print("X:", rotate_x)

quat = tf.transformations.quaternion_from_euler(rotate_z, rotate_x,0,'rzxy')
print(quat)