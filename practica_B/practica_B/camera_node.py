import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)

        ## DESCOMENTAR TODO ESTO PARA USAR LA CAMARA DEL ORDENADOR
        self.get_logger().info("Camera node initialized")
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open the camera.")
            raise Exception("Failed to open the camera.")
        
        self.bridge = CvBridge()

        self.timer = self.create_timer(0.0018, self.timer_callback)  # 1/0.05 = 20 FPS

    def timer_callback(self):
        ret, frame = self.cap.read()
        # cv2.imshow("Camera", frame)
        # cv2.waitKey(1)
        ret = True

        ## DESCOMENTAR ESTO PARA "FALSEAR" LA CAMARA
        # frame = np.load("assets/frame.npy")

        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
            self.get_logger().info('Image published')
        else:
            self.get_logger().warn('Failed to read the image from the camera.')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
