import numpy as np
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from battery_assembly_interfaces.srv import RequestPathPlanningExecution

# Rest of your code remains the same
T_b_c = np.array([
 [ 9.99999618e-01,  8.72420101e-04,  4.91172605e-05, -5.65186938e-01],
 [ 8.72481761e-04, -9.99998813e-01, -1.26966881e-03,  7.44921756e-04],
 [ 4.80095176e-05,  1.26971118e-03, -9.99999193e-01,  5.99694280e-02],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

# Transformation from camera to a point in the camera view (battery holder part)
# TODO : receive matrix from camera subscriber
T_c_p = np.array([
 [ 9.87492722e-01, -4.54053250e-03, -1.57599200e-01, -6.21657633e-02],
 [-0.00000000e+00, 9.99585232e-01, -2.87986819e-02, -1.13597787e-02],
 [ 1.57664594e-01, 2.84384888e-02, 9.87083141e-01, 3.89359698e-01],
 [ 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
])

class RequestPathPlanningExecutionNode(Node):
    def __init__(self):
        super().__init__('request_path_planning_execution')
        self.cli = self.create_client(RequestPathPlanningExecution, 'request_path_planning_execution')
        while not self.cli.wait_for_service(timeout_sec=20.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = RequestPathPlanningExecution.Request()

    def send_request(self, object_name, transformation_matrix):
        self.req.object_name = object_name
        self.req.transformation_matrix = transformation_matrix.flatten().tolist()
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client = RequestPathPlanningExecutionNode()
    object_name = "example_object"

    client.get_logger().info(f'Ready to receive and send battery cell pose.')
    
    # The transformation of the base plate of the robot to the specified point in the camera
    T_b_p = np.dot(T_b_c, T_c_p)

    client.get_logger().info(f'Calculated pose, sending it to the path planning node.')
    
    response = client.send_request(object_name, T_b_p)
    if response.success:
        client.get_logger().info(f'Successfully requested path planning execution for {object_name}')
        client.get_logger().info(f'Transformation matrix sent:\n{T_b_p}')
    else:
        client.get_logger().warn(f'Failed to request path planning execution for {object_name}')
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()