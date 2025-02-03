import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import time
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Declare parameters with default values
        self.declare_parameters(namespace='',
            parameters=[
                ('camera_device', '/dev/video0'),
                ('frame_width', 640),
                ('frame_height', 480),
                ('fps', 30),
                ('show_display', False),
                ('save_images', False),
                ('confidence_threshold', 0.5),
                ('email_sender', 'alkhalilsiddick95@gmail.com'), 
                ('email_password', 'khalil95'),      
                ('email_receiver', 'alkhalilsiddick26@gmail.com'),
                ('smtp_server', 'smtp.gmail.com'),
                ('smtp_port', 587)
            ]
        )

        # Create publisher
        self.publisher = self.create_publisher(Image, 'detection_output', 10)

        # Initialize video capture
        self.initialize_camera()

        # Load model and configuration
        self.initialize_model()

        # Processing timer
        self.timer = self.create_timer(1.0 / self.get_parameter('fps').value, self.process_frame)

        self.get_logger().info("Node initialized successfully")

    def initialize_camera(self):
        """Initialize camera with parameterized settings"""
        try:
            camera_device = self.get_parameter('camera_device').value
            frame_width = self.get_parameter('frame_width').value
            frame_height = self.get_parameter('frame_height').value
            fps = self.get_parameter('fps').value

            # Try opening camera
            self.cap = cv2.VideoCapture(camera_device)
            if not self.cap.isOpened():
                raise RuntimeError("Failed to open camera device")

            # Set camera properties
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
            self.cap.set(cv2.CAP_PROP_FPS, fps)

            self.get_logger().info(f"Camera initialized with device: {camera_device}")

        except Exception as e:
            self.log_camera_devices()
            self.get_logger().error(f"Camera initialization failed: {str(e)}")
            raise RuntimeError("Camera initialization failed") from e

    def log_camera_devices(self):
        """Log available camera devices"""
        self.get_logger().info("Available camera devices:")
        for i in range(5):  # Check the first 5 devices
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                self.get_logger().info(f"Device {i}: {cap.getBackendName()}")
            cap.release()

    def initialize_model(self):
        """Initialize object detection model"""
        try:
            package_dir = get_package_share_directory('smart_car_pkg')
            model_dir = os.path.join(package_dir, 'models')

            # Model files configuration
            self.required_files = {
                'pb': 'frozen_inference_graph.pb',
                'pbtxt': 'ssd_mobilenet_v2_coco_2018_03_29.pbtxt',
                'labels': 'object_detection_classes_coco.txt'
            }

            # Verify files exist
            missing_files = [f for f in self.required_files.values()
                             if not os.path.exists(os.path.join(model_dir, f))]

            if missing_files:
                self.get_logger().error(f"Missing model files: {', '.join(missing_files)}")
                raise FileNotFoundError(f"Missing model files: {missing_files}")

            # Configure network
            self.net = cv2.dnn.readNetFromTensorflow(
                os.path.join(model_dir, self.required_files['pb']),
                os.path.join(model_dir, self.required_files['pbtxt'])
            )
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

            # Load class labels
            with open(os.path.join(model_dir, self.required_files['labels']), 'r') as f:
                self.classes = [line.strip() for line in f.readlines()]

            self.get_logger().info(f"Model initialized with {len(self.classes)} classes")

        except Exception as e:
            self.get_logger().error(f"Model initialization failed: {str(e)}")
            raise RuntimeError("Model initialization failed") from e

    def send_email(self, subject, body):
        """Send an email notification"""
        sender_email = self.get_parameter('email_sender').value
        receiver_email = self.get_parameter('email_receiver').value
        password = self.get_parameter('email_password').value
        smtp_server = self.get_parameter('smtp_server').value
        smtp_port = self.get_parameter('smtp_port').value

        message = MIMEMultipart()
        message["From"] = sender_email
        message["To"] = receiver_email
        message["Subject"] = subject

        message.attach(MIMEText(body, "plain"))

        try:
            server = smtplib.SMTP(smtp_server, smtp_port)
            server.starttls()
            server.login(sender_email, password)
            text = message.as_string()
            server.sendmail(sender_email, receiver_email, text)
            server.quit()
            self.get_logger().info("Email notification sent successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to send email: {str(e)}")

    def process_frame(self):
        """Process each video frame"""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to grab frame")
            return

        # Timer for frame processing
        start_time = time.time()

        # Create a blob from the frame
        blob = cv2.dnn.blobFromImage(frame, 1.0, (300, 300), (104.0, 177.0, 123.0), swapRB=True, crop=False)
        self.net.setInput(blob)

        # Perform inference
        detections = self.net.forward()

        # Post-process the detections
        height, width = frame.shape[:2]
        class_ids = []
        confidences = []
        boxes = []

        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > self.get_parameter('confidence_threshold').value:
                class_id = int(detections[0, 0, i, 1])
                box = detections[0, 0, i, 3:7] * np.array([width, height, width, height])
                (x, y, x2, y2) = box.astype('int')

                # Save the detection info
                class_ids.append(class_id)
                confidences.append(float(confidence))
                boxes.append((x, y, x2, y2))

        # Non-maxima suppression
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.get_parameter('confidence_threshold').value, 0.4)

        # Draw bounding boxes and labels
        if len(indices) > 0:
            for i in indices.flatten():
                (x, y, x2, y2) = boxes[i]
                label = self.classes[class_ids[i]]
                confidence = confidences[i]
                color = (0, 255, 0)  # Green for bounding box
                cv2.rectangle(frame, (x, y), (x2, y2), color, 2)
                cv2.putText(frame, f"{label}: {confidence:.2f}", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # Send email notification for specific objects
                if label == "person":
                    subject = "Surveillance Alert"
                    body = f"Detected {label} with confidence {confidence:.2f}"
                    self.send_email(subject, body)

        # Display the frame
        if self.get_parameter('show_display').value:
            cv2.imshow('Object Detection', frame)
            cv2.waitKey(1)

        # Convert to ROS Image message and publish
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher.publish(ros_image)

        # Log processing time per frame
        processing_time = time.time() - start_time
        self.get_logger().info(f"Processed frame in {processing_time:.4f} seconds")

    def shutdown(self):
        """Clean up resources before shutting down the node"""
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()