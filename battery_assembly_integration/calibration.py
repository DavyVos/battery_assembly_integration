import numpy as np

c_camera_min = np.array([50.5, -2.5, 505])

c_camera_max = np.array([60.5, -12.5, 515])

tcp_ee_min = np.array([-551.55, -64.85, 105.49])

tcp_ee_max = np.array([-461.55, -63.85, 215.49])

def map_value(min_one, max_one, min_two, max_two, original_value):
    return (((original_value - min_one) * (max_two - max_one)) / (max_one - min_one)) + min_two

def scale_to_arm(p_camera):
    return np.array([
        map_value(c_camera_min[0], c_camera_max[0], tcp_ee_min[0], tcp_ee_max[0], p_camera[0]),
        map_value(c_camera_min[1], c_camera_max[1], tcp_ee_min[1], tcp_ee_max[1], p_camera[1]),
        map_value(c_camera_min[2], c_camera_max[2], tcp_ee_min[2], tcp_ee_max[2], p_camera[2]),
    ])

c_camera = c_camera_min

original_point = scale_to_arm(c_camera)
print(original_point)