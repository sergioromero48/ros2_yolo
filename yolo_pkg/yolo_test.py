import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from std_msgs.msg import String  # For publishing detection results as JSON
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
from ultralytics import YOLO  # YOLO model for object detection
import os
import json  # For JSON serialization

class YoloDetectionNode(Node):
    """
    A ROS 2 Node that subscribes to an image topic, runs YOLO object detection,
    publishes detection data as JSON, and publishes the annotated image.
    """
    def __init__(self):
        """
        Class constructor to set up the node.
        """
        # Initialize the Node class's constructor and give it a name
        super().__init__('yolo_detection_node')
        
        # Create the subscriber to receive an Image from the /camera/image_raw topic
        self.subscription = self.create_subscription(
            Image, 
            '/camera/image_raw',
            self.listener_callback, 
            10)
        self.subscription  # prevent unused variable warning

        # Publishers for detection results and annotated image
        self.detection_publisher = self.create_publisher(String, '/camera/full_detections', 10)
        self.image_publisher = self.create_publisher(Image, '/camera/image_annotated', 10)
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Load the YOLO model path
        model_path = os.path.join(
            os.getenv('AMENT_PREFIX_PATH').split(':')[0],  # Get the ROS2 install path
            'share',                                      # The share directory
            'yolo_pkg',                                   # Your package directory
            'yolo_models',                                # The folder with the model
            'yolo11n.pt'                                  # Model filename
        )

        # Check if the model file exists
        if not os.path.isfile(model_path):
            self.get_logger().error(f"Model file not found at {model_path}")
        else:
            self.model = YOLO(model_path)
       
    def listener_callback(self, data):
        """
        Callback function that processes an image and publishes all YOLO detection data.
        """
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        current_frame_rgb = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)

        # Run YOLO on the image
        results = self.model(current_frame_rgb)

        # Generate the annotated image
        result_image = results[0].plot()

        # Convert annotated image to ROS Image message and publish it
        annotated_image_msg = self.br.cv2_to_imgmsg(result_image, encoding='bgr8')
        self.image_publisher.publish(annotated_image_msg)

        # Prepare detection data dictionary
        full_results = {
            "detections": []
        }

        # Process each result from YOLO
        for detection in results:
            detection_data = {
                "boxes": detection.boxes.data.cpu().numpy().tolist(),  # Bounding boxes
                "masks": detection.masks.data.cpu().numpy().tolist() if detection.masks is not None else None,  # Segmentation masks
                "keypoints": detection.keypoints.data.cpu().numpy().tolist() if detection.keypoints is not None else None,  # Pose keypoints
                "probs": detection.probs.data.cpu().numpy().tolist() if detection.probs is not None else None,  # Classification probabilities
                "obb": detection.obb.data.cpu().numpy().tolist() if detection.obb is not None else None  # Oriented bounding boxes
            }

            full_results["detections"].append(detection_data)

        # Publish all detection data as a JSON string
        detection_msg = String()
        detection_msg.data = json.dumps(full_results)
        self.detection_publisher.publish(detection_msg)

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create the node
    yolo_detection_node = YoloDetectionNode()
    
    # Spin the node so the callback functions are called
    rclpy.spin(yolo_detection_node)
    
    # Destroy the node explicitly
    yolo_detection_node.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
