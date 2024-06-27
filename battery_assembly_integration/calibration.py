import numpy as np
from scipy.spatial.transform import Rotation as R


def forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6):
    # DH parameters
    d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996]
    a = [0, -0.425, -0.3922, 0, 0, 0]
    alpha = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
    
    # Base transformation matrix
    T = np.eye(4)
    
    # Calculate transformation matrices for each joint
    for i in range(6):
        theta = [theta1, theta2, theta3, theta4, theta5, theta6][i]
        
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha[i])
        sa = np.sin(alpha[i])
        
        A = np.array([
            [ct, -st*ca, st*sa, a[i]*ct],
            [st, ct*ca, -ct*sa, a[i]*st],
            [0, sa, ca, d[i]],
            [0, 0, 0, 1]
        ])
        
        T = np.dot(T, A)
    
    # Extract position and orientation from the final transformation matrix
    position = T[:3, 3]
    orientation = T[:3, :3]
    
    return T

# Calculate forward kinematics of the ur5e
theta1, theta2, theta3, theta4, theta5, theta6 = np.deg2rad(-92), np.deg2rad(-99), np.deg2rad(-126), np.deg2rad(-46), np.deg2rad(91), np.deg2rad(-2)  # Joint angles in radians
T_b_ee = forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6)

# Transformation matrix from camera to end-effector
T_c_ee = np.array([
    [0.00000000e+00, 9.99941254e-01, -1.08392132e-02, -4.14540947e-03],
    [-7.60767771e-01, 7.03491157e-03, 6.48986062e-01, 2.48201868e-01],
    [6.49024190e-01, 8.24612407e-03, 7.60723079e-01, 2.90935199e-01],
    [0, 0, 0, 1]
])

# Rotation matrix from end effector to camera
R_ee_c = np.transpose(T_c_ee[:3, :3])
# Calculate the inverse of 
# the translation vector for end effector to camera
t_ee_c = np.dot(-R_ee_c, T_c_ee[:3, 3])
# Compose the inverse transformation matrix from end-effector to camera
T_ee_c = np.eye(4,4)
T_ee_c[:3, :3] = R_ee_c
T_ee_c[:3, 3] = t_ee_c

# the transformation matrix for the baseplate of the robot to the camera is
# the transformation from baseplate to end-effector x end-effector to camera
T_b_c = np.dot(T_b_ee, T_ee_c)

R_b_ee = T_b_ee[:3,:3]
# Convert the rotation matrix to a quaternion
r = R.from_matrix(R_b_ee)
quaternion = r.as_euler('xyz', degrees=False)


# Rotation matrix from camera to the base plate
R_c_b = np.transpose(T_b_c[:3, :3])
# Calculate the inverse of 
# the translation vector for camera to the base plate
t_c_b = np.dot(-R_c_b, T_b_c[:3, 3])
# Compose the inverse transformation matrix from end-effector to camera
T_c_b = np.eye(4,4)
T_c_b[:3, :3] = R_c_b
T_c_b[:3, 3] = t_c_b

print("Quaternion of end effector:", quaternion)
print("Base plate to end-effector transformation:\n", T_b_ee)
print("End-effector to camera transformation:\n", T_ee_c)
print("Base plate to camera transformation:\n", T_b_c)
print("Camera to base plate transformation:\n", T_c_b)