import rclpy
from rclpy.node import Node
from example_interfaces.msg import String, Bool
from sensor_msgs.msg import Image
from ultralytics import YOLO
from cv_bridge import CvBridge
import cv2
from person_msg.msg import PersonInfo  # Import the custom message
from geometry_msgs.msg import Point  # Import geometry_msgs/Point
from std_msgs.msg import Bool  # For the bool field

class person_recon(Node):

    def __init__(self):
        super().__init__("person_recon")

        self.model = YOLO("yolov8n.pt")

        self.declare_parameter("log_level", 10)
        self.add_post_set_parameters_callback(self.post_set_callback)

        self.logger = self.get_logger()
        self.logger.set_level(10)
        
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(PersonInfo, "person_info", 10)

        self.sub = self.create_subscription(Image, "camera/image_raw", self.callback_subscriber, qos_profile=10)
        # Initialize with default values
        self.publish_flag(False, (0.0, 0.0, 0.0))

    def add_bounding_box(self, frame, results):
        coordinates = None
        for result in results:
            boxes = result.boxes
            for box in boxes:
                if box.cls == 0:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    coordinates = (float(x1), float(y1), 0.0)
        return frame, coordinates

    def post_set_callback(self, parameters):
        for param in parameters:
            self.logger.info(f"Parametro modificado {param}")
            if param.name == "log_level":
                self.logger.set_level(param.value)

    def callback_subscriber(self, msg = None):
        if msg is None:
            msg = "/home/alumno/Descargas/Imagen pegada.png"

        self.logger.info(f"Received image")
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        try:
            results = self.model(frame,classes=0,verbose=False,conf=0.9)
        except FileNotFoundError:
            return

        exists_person = any(box.cls == 0 for box in results[0].boxes)
        self.logger.info(str(exists_person))

        frame_annotated, coordinates = self.add_bounding_box(frame, results)
        # Flip the frame horizontally
        frame_annotated = cv2.flip(frame_annotated, 1)
        cv2.imshow("Frame", frame_annotated)
        cv2.waitKey(1)

        self.publish_flag(exists_person, coordinates)

    def publish_flag(self, state: bool, coordinates: tuple):
        msg = PersonInfo()
        msg.person_exists = state
        if coordinates is None:
            coordinates = (0.0, 0.0, 0.0)
        msg.coordinates = Point(x=float(coordinates[0]), y=float(coordinates[1]), z=float(coordinates[2]))
        self.logger.debug(f"Publishing flag {state}")
        self.publisher.publish(msg)

def main(args=None):
    try:
        rclpy.init(args=args)
        node = person_recon()
        rclpy.spin(node) 
    except KeyboardInterrupt:
        print("Finalizado el nodo talker")
    finally:
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()