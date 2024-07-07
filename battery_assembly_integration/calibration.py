import numpy as np
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        self.calculate_and_publish()

    def calculate_and_publish(self):
        T_b_c = self.calculateRobotArmToCameraTransform()
        self.get_logger().info(f"Base plate to camera transformation:\n{T_b_c}")

    def forward_kinematics(self, theta1, theta2, theta3, theta4, theta5, theta6):
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

    def calculateRobotArmToCameraTransform(self):
        # Calculate forward kinematics of the ur5e
        theta1, theta2, theta3, theta4, theta5, theta6 = np.deg2rad(-13.73), np.deg2rad(-52.83), np.deg2rad(113.39), np.deg2rad(-150.54), np.deg2rad(-89.93), np.deg2rad(76.22) # Joint angles in radians
        T_b_ee = self.forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6)

        # Transformation matrix from camera to end-effector (calibration point)
        T_c_ee = np.array([
            [ 9.99701490e-01, -7.67433262e-04, -2.44201223e-02, 5.66600000e-02],
            [ 0.00000000e+00, 9.99506560e-01, -3.14107591e-02, 1.38000000e-02],
            [ 2.44321782e-02, 3.14013826e-02, 9.99208197e-01, 4.47850000e-01],
            [ 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
        ])

        # Rotation matrix from end-effector to camera
        R_ee_c = np.transpose(T_c_ee[:3, :3])
        # Calculate the inverse of the translation vector for end effector to camera
        t_ee_c = np.dot(-R_ee_c, T_c_ee[:3, 3])
        # Compose the inverse transformation matrix from end-effector to camera
        T_ee_c = np.eye(4,4)
        T_ee_c[:3, :3] = R_ee_c
        T_ee_c[:3, 3] = t_ee_c

        # The transformation matrix for the baseplate of the robot to the camera is
        # the transformation from baseplate to end-effector x end-effector to camera
        T_b_c = np.dot(T_b_ee, T_ee_c)

        # Rotation matrix from the base plate of the robot to the end-effector
        R_b_ee = T_b_ee[:3,:3]
        # Convert the rotation matrix to a quaternion
        r = R.from_matrix(R_b_ee)
        quaternion = r.as_euler('xyz', degrees=False)

        # Rotation matrix from camera to the base plate
        R_c_b = np.transpose(T_b_c[:3, :3])
        # Calculate the inverse of the translation vector for camera to the base plate
        t_c_b = np.dot(-R_c_b, T_b_c[:3, 3])
        # Compose the inverse transformation matrix from end-effector to camera
        T_c_b = np.eye(4,4)
        T_c_b[:3, :3] = R_c_b
        T_c_b[:3, 3] = t_c_b

        self.get_logger().info(f"Euler angles of the end-effector: {quaternion}")
        self.get_logger().info(f"Base plate to end-effector transformation:\n{T_b_ee}")
        self.get_logger().info(f"End-effector to camera transformation:\n{T_ee_c}")
        self.get_logger().info(f"Base plate to camera transformation:\n{T_b_c}")
        self.get_logger().info(f"Camera to base plate transformation:\n{T_c_b}")

        # Sanity check to verify that the correct inverse is calculated
        self.get_logger().info(f"Sanity check:\n{np.dot(T_b_c, T_c_b)}")

        return T_b_c

def main(args=None):
    rclpy.init(args=args)
    calibration_node = CalibrationNode()
    rclpy.spin_once(calibration_node)
    calibration_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()