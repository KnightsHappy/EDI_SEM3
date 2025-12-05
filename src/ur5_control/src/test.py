from scipy.spatial.transform import Rotation as R
import numpy as np

# quaternion in [x, y, z, w] format
quat = np.array([0.029, 0.98, 0.045, 0.033])

# convert to rotation object
r = R.from_quat(quat)

# get Euler angles in degrees, using standard "xyz" (roll, pitch, yaw)
euler_deg = r.as_euler('xyz', degrees=True)
print("Roll, Pitch, Yaw (degrees):", euler_deg)
