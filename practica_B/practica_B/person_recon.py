import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point  # Import geometry_msgs/Point
from person_msg.msg import PersonInfo  # Import the custom message
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO


class person_recon(Node):

    def __init__(self):
        super().__init__("person_recon")
        self.logger = self.get_logger()

        # Declare global logger level parameter
        self.declare_parameter("global_log_level", 20)  # Default to DEBUG level
        self.add_post_set_parameters_callback(self.parameters_callback)

        # Set initial log level from parameter
        self.logger.set_level(self.get_parameter("global_log_level").value)

        self.logger.info("Initializing Person Recognition Node...")

        try:
            self.model = YOLO("yolov8n.pt")
            self.logger.info("YOLO model loaded successfully")
        except Exception as e:
            self.logger.error(f"Failed to load YOLO model: {str(e)}")
            raise

        self.bridge = CvBridge()
        self.logger.debug("CV Bridge initialized")

        self.publisher = self.create_publisher(PersonInfo, "person_info", 10)
        self.logger.debug("Created publisher for 'person_info' topic")

        self.create_subscription(
            Image, "camera/image_raw", self.callback_subscriber, qos_profile=10
        )
        self.logger.debug("Created subscription to 'camera/image_raw' topic")

        # Initialize with default values
        self.publish_flag(False, (0.0, 0.0, 0.0))
        self.logger.info("Person Recognition Node successfully initialized")

    def parameters_callback(self, parameters):
        """Callback for parameter changes"""
        for param in parameters:
            if param.name == "global_log_level":
                self.logger.set_level(param.value)
                self.logger.info(f"Log level changed to: {param.value}")

    def add_bounding_box(self, frame, results):
        coordinates = (0.0, 0.0, 0.0)
        for result in results:
            boxes = result.boxes
            for box in boxes:
                if box.cls == 0:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    coordinates = (float(x1), float(y1), 0.0)
                    self.logger.debug(f"Person detected at coordinates: ({x1}, {y1})")
        return frame, coordinates

    def callback_subscriber(self, msg=None):
        if msg is None:
            self.logger.warn("No image message received, using default image")
            msg = "/home/alumno/Descargas/Imagen pegada.png"

        self.logger.debug("Processing new image frame")
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.logger.error(f"Failed to convert image message: {str(e)}")
            return

        try:
            results = self.model(frame, classes=0, verbose=False, conf=0.9)
        except FileNotFoundError:
            self.logger.error("YOLO model file not found")
            return
        except Exception as e:
            self.logger.error(f"Error during person detection: {str(e)}")
            return

        exists_person = any(box.cls == 0 for box in results[0].boxes)
        self.logger.debug(f"Person detection result: {exists_person}")

        frame_annotated, coordinates = self.add_bounding_box(frame, results)
        # Flip the frame horizontally
        frame_annotated = cv2.flip(frame_annotated, 1)
        cv2.imshow("Frame", frame_annotated)
        cv2.waitKey(1)

        self.publish_flag(exists_person, coordinates)

    def publish_flag(self, state: bool, coordinates: tuple):
        msg = PersonInfo()
        msg.person_exists = state
        x, y, z = coordinates
        msg.coordinates = Point(
            x=float(x), y=float(y), z=float(z)
        )
        self.logger.debug(
            f"Publishing person detection status: {state} at coordinates: {coordinates}"
        )
        self.publisher.publish(msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = person_recon()
        node.logger.info("Starting Person Recognition Node...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.logger.info("Keyboard interrupt received, shutting down...")
    except Exception as e:
        node.logger.error(f"An error occurred: {str(e)}")
    finally:
        rclpy.try_shutdown()
        node.logger.info("Person Recognition Node shutdown complete")


if __name__ == "__main__":
    main()
