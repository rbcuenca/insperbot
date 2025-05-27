import rclpy
import numpy as np
import cv2
import json
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from apriltag_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import String
from robcomp_interfaces.msg import AprilTagInsper


class AprilTagDistancePublisher(Node):
    def __init__(self):
        super().__init__('april_tag_distance_publisher')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.publisher_ = self.create_publisher(AprilTagInsper, 'april_tag', 10)

        # Parâmetros da câmera (exemplo, substitua pelos seus valores calibrados)
        self.camera_matrix = np.array([[600, 0, 320],
                                       [0, 600, 240],
                                       [0, 0, 1]], dtype=float)
        self.dist_coeffs = np.zeros((5, 1))  # Supondo sem distorção

        # Tamanho do lado do marcador em metros (ajuste conforme seu marcador)
        

        # Pontos 3D do marcador no sistema do mundo (origem no centro do marcador)


    def listener_callback(self, msg):
        for detection in msg.detections:
            tag_id = detection.id
            center_x = detection.centre.x
            center_y = detection.centre.y
            
            if tag_id < 10: self.tag_size = 0.05
            else: self.tag_size = 0.20
            
            half_size = self.tag_size / 2.0
            self.object_points = np.array([
                [-half_size,  half_size, 0],
                [ half_size,  half_size, 0],
                [ half_size, -half_size, 0],
                [-half_size, -half_size, 0]
            ])

            # Extrai os cantos 2D do marcador
            image_points = np.array([
                [detection.corners[0].x, detection.corners[0].y],
                [detection.corners[1].x, detection.corners[1].y],
                [detection.corners[2].x, detection.corners[2].y],
                [detection.corners[3].x, detection.corners[3].y]
            ], dtype=float)
            # Calcula pose usando solvePnP
            success, rvec, tvec = cv2.solvePnP(
                self.object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )

            if success:
                distance = np.linalg.norm(tvec)
            else:
                distance = float('nan')

            # info_dict = {
            #     'ID': int(tag_id),
            #     'CX': int(center_x),
            #     'CY': int(center_y),
            #     'DIST': distance
            # }
            # self.get_logger().info(json.dumps(info_dict))

            msg_out = AprilTagInsper()
            msg_out.id = tag_id
            msg_out.cx = center_x
            msg_out.cy = center_y
            msg_out.dist = distance
            print(msg_out)
            self.publisher_.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDistancePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
