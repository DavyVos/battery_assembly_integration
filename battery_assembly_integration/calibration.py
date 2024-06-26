import numpy as np

# Center of the circle as detected by the camera
c_camera = np.array([50.5, -2.5, 505])

# roll, pitch and yaw of end-effector
x = np.deg2rad(177.61)
y = np.deg2rad(-3.32)
z = np.deg2rad(98.26)

# Pure rotations of the robot arm end effector
ee_R_x = np.array([[1,0,0],
                    [0, np.cos(x), -np.sin(x)],
                    [0, np.sin(x), np.cos(x)]])

ee_R_y = np.array([[np.cos(y), 0, np.sin(y)],
                    [0, 1, 0],
                    [-np.sin(y), 0, np.cos(y)]])

ee_R_z = np.array([[np.cos(z), -np.sin(z), 0],
                    [np.sin(z), np.cos(z), 0],
                    [0, 0, 1]])

# End effector rotation matrix
R = np.dot(ee_R_z, np.dot(ee_R_y, ee_R_x))

# vector values towards tcp (tool center point)
x_tcp, y_tcp, z_tcp = -551.55, -63.85, 105.49
t_base_to_ee  = np.array([x_tcp, y_tcp, z_tcp])

# Construct the transformation matrix from the base to the end-effector
T_base_to_ee = np.eye(4)
T_base_to_ee[:3, :3] = R
T_base_to_ee[:3, 3] = t_base_to_ee

# Vector from end-effector to camera
t_ee_to_cam = -c_camera

# Construct the transformation matrix from the end-effector to the camera
T_ee_to_cam = np.eye(4)
T_ee_to_cam[:3, 3] = t_ee_to_cam

# Calculate the transformation matrix from the base to the camera
T_base_to_cam = np.dot(T_base_to_ee, T_ee_to_cam)

# Print the resulting position vector in the base frame
print("Position of Camera in Base Frame:")
print(T_base_to_cam )

# Example point in the camera frame
p_cam = np.array([50.5, -2.5, 505, 1])  # Homogeneous coordinates

# Transform the point from camera frame to base frame
p_base = np.dot(T_base_to_cam , p_cam)

# Print the resulting point in the base frame
print("Point in Base Frame:")
print(p_base[:3])  # Ignore the homogeneous coordinate
