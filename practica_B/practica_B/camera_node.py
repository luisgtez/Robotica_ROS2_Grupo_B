import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")
        self.logger = self.get_logger()

        # Declare global logger level parameter
        self.declare_parameter("global_log_level", 20)  # Default to DEBUG level
        # Declare frequency parameter
        self.declare_parameter("frequency", 10.0)  # Default to 10 Hz
        self.add_post_set_parameters_callback(self.parameters_callback)

        # Set initial log level from parameter
        self.logger.set_level(self.get_parameter("global_log_level").value)

        self.logger.info("Initializing Camera Publisher Node...")

        self.publisher_ = self.create_publisher(Image, "camera/image_raw", 10)
        self.logger.debug("Created publisher for 'camera/image_raw' topic")

        self.logger.info("Attempting to open camera...")
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.logger.error("Failed to open the camera.")
            raise Exception("Failed to open the camera.")
        self.logger.info("Camera opened successfully")

        self.bridge = CvBridge()
        self.logger.debug("CV Bridge initialized")

        # Get frequency from parameter
        frequency = self.get_parameter("frequency").value
        self.timer = self.create_timer(1.0 / frequency, self.timer_callback)
        self.logger.debug(f"Created timer with {frequency}Hz frequency")
        self.logger.info("Camera Publisher Node successfully initialized")

    def parameters_callback(self, parameters):
        """Callback for parameter changes"""
        for param in parameters:
            if param.name == "global_log_level":
                self.logger.set_level(param.value)
                self.logger.info(f"Log level changed to: {param.value}")
            elif param.name == "frequency":
                self.destroy_timer(self.timer)
                self.timer = self.create_timer(1.0 / param.value, self.timer_callback)
                self.logger.info(f"Camera frequency changed to: {param.value} Hz")

    def timer_callback(self):
        ret, frame = self.cap.read()
        ret = True

        if ret:
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.publisher_.publish(msg)
                self.logger.debug("Image frame published successfully")
            except Exception as e:
                self.logger.error(f"Failed to convert and publish image: {str(e)}")
        else:
            self.logger.warn("Failed to read frame from camera")

    def destroy_node(self):
        self.logger.info("Releasing camera resources...")
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        node.logger.info("Starting Camera Publisher Node...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.logger.info("Keyboard interrupt received, shutting down...")
    except Exception as e:
        node.logger.error(f"An error occurred: {str(e)}")
    finally:
        node.logger.info("Cleaning up resources...")
        node.destroy_node()
        rclpy.shutdown()
        node.logger.info("Camera Publisher Node shutdown complete")


if __name__ == "__main__":
    main()
