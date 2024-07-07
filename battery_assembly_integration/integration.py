import numpy as np
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
import numpy as np
from example_interfaces.srv import RequestObject


def forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6):
    # DH parameters (ur5e)
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
theta1, theta2, theta3, theta4, theta5, theta6 = np.deg2rad(-13.73), np.deg2rad(-52.83), np.deg2rad(113.39), np.deg2rad(-150.54), np.deg2rad(-89.93), np.deg2rad(76.22)  # Joint angles in radians
T_b_ee = forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6)

# Transformation matrix from camera to end-effector (calibration point)
T_c_ee = np.array([
    [ 9.99701490e-01, -7.67433262e-04, -2.44201223e-02,  5.66600000e-02],
    [ 0.00000000e+00,  9.99506560e-01, -3.14107591e-02,  1.38000000e-02],
    [ 2.44321782e-02,  3.14013826e-02,  9.99208197e-01,  4.47850000e-01],
    [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
])

def calculateRobotArmToCameraTransform():
    # Rotation matrix from end-effector to camera
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

    # Rotation matrix from the base plate of the robot to the end-effector
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

    print("Eular angles of the end-effector:", quaternion)
    print("Base plate to end-effector transformation:\n", T_b_ee)
    print("End-effector to camera transformation:\n", T_ee_c)
    print("Base plate to camera transformation:\n", T_b_c)
    print("Camera to base plate transformation:\n", T_c_b)
    # Sanity check to verify that the correct inverse is calculated
    print(np.dot(T_b_c, T_c_b))
    
    return T_b_c
    
    
# Transformation from camera to a point in the camera view (battery holder part)
# TODO : receive matrix from camera subscriber
T_c_p = np.array([
        [ 9.87492722e-01, -4.54053250e-03, -1.57599200e-01, -6.21657633e-02],
        [-0.00000000e+00,  9.99585232e-01, -2.87986819e-02, -1.13597787e-02],
        [ 1.57664594e-01,  2.84384888e-02,  9.87083141e-01,  3.89359698e-01],
        [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
    ])

# Perform the calculation for calibration and return the transformation from base of the robot to camera
T_b_c = calculateRobotArmToCameraTransform()
    
# The transformation of the base plate of the robot to the specfied point in the camera
T_b_p = np.dot(T_b_c, T_c_p)

class ObjectPoseClient(Node):

    def __init__(self):
        super().__init__('object_pose_client')
        self.cli = self.create_client(ObjectPose, 'get_object_pose')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ObjectPose.Request()

    def send_request(self, object_name):
        self.req.object_name = object_name
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    object_pose_client = ObjectPoseClient()
    object_name = "example_object"
    
    response = object_pose_client.send_request(object_name)

    if response.success:
        transform_matrix = np.array(response.transformation_matrix).reshape(4, 4)
        object_pose_client.get_logger().info(f'Received transformation matrix for {object_name}:')
        object_pose_client.get_logger().info(f'\n{transform_matrix}')
    else:
        object_pose_client.get_logger().warn(f'Failed to get pose: {response.message}')

    object_pose_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()class ObjectPoseClient(Node):

    def __init__(self):
        super().__init__('object_pose_client')
        self.cli = self.create_client(ObjectPose, 'get_object_pose')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ObjectPose.Request()

    def send_request(self, object_name):
        self.req.object_name = object_name
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    object_pose_client = ObjectPoseClient()
    object_name = "example_object"
    
    response = object_pose_client.send_request(object_name)

    if response.success:
        transform_matrix = np.array(response.transformation_matrix).reshape(4, 4)
        object_pose_client.get_logger().info(f'Received transformation matrix for {object_name}:')
        object_pose_client.get_logger().info(f'\n{transform_matrix}')
    else:
        object_pose_client.get_logger().warn(f'Failed to get pose: {response.message}')

    object_pose_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()