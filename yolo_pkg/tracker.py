import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # To receive annotated image data
from std_msgs.msg import String  # To receive detection data
import json  # For JSON parsing
import numpy as np  # For efficient numerical calculations
import cv2  # OpenCV for drawing and displaying
from cv_bridge import CvBridge  # For converting between ROS and OpenCV images

class TrackerNode(Node):
    """
    A ROS 2 Node that subscribes to detection data and annotated images,
    calculates centroids and distances, and visualizes tracking.
    """
    def __init__(self, screen_width=640, screen_height=480, display_width=640, display_height=480):
        """
        Class constructor to set up the node.
        :param screen_width: Width of the screen or frame
        :param screen_height: Height of the screen or frame
        :param display_width: Width of the display window
        :param display_height: Height of the display window
        """
        # Initialize the Node class's constructor and give it a name
        super().__init__('tracker_node')
        
        # Subscribe to the detection data
        self.detection_subscription = self.create_subscription(
            String, 
            '/camera/full_detections',
            self.detection_callback, 
            10)
        
        # Subscribe to the annotated image data
        self.image_subscription = self.create_subscription(
            Image, 
            '/camera/image_annotated',
            self.image_callback, 
            10)

        # Bridge to convert ROS images to OpenCV
        self.br = CvBridge()

        # Screen center as a numpy array
        self.screen_center = np.array([screen_width / 2, screen_height / 2])

        # Frame and display dimensions
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.display_width = display_width
        self.display_height = display_height

        # Store current detections and frame
        self.current_detections = []
        self.current_frame = None

    def detection_callback(self, msg):
        """
        Callback function that processes detection data.
        """
        try:
            # Parse the JSON data
            full_results = json.loads(msg.data)
            self.current_detections = full_results.get("detections", [])
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON data: {e}")

    def image_callback(self, msg):
        """
        Callback function that processes the annotated image and performs visualization.
        """
        # Convert ROS Image message to OpenCV image
        self.current_frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Proceed if there are detections to draw
        if self.current_detections:
            # Draw detections on the frame
            for detection in self.current_detections:
                boxes = detection.get("boxes", [])
                
                for box in boxes:
                    x1, y1, x2, y2, confidence, class_id = box  # Assuming box format [x1, y1, x2, y2, confidence, class_id]

                    # Filter for persons with confidence > 80
                    if class_id == 0 and confidence > 0.80:  # Assuming '0' is the class ID for 'person'
                        # Calculate centroid
                        centroid = np.array([(x1 + x2) / 2, (y1 + y2) / 2])
                        
                        # Draw line from centroid to screen center in white
                        cv2.line(self.current_frame, 
                                 (int(centroid[0]), int(centroid[1])), 
                                 (int(self.screen_center[0]), int(self.screen_center[1])), 
                                 (255, 255, 255), 2)  # White color for the line

                        # Calculate distance and display it
                        distance = np.linalg.norm(centroid - self.screen_center)
                        label_text = f"Dist: {distance:.2f}"
                        cv2.putText(self.current_frame, label_text, 
                                    (int(centroid[0]), int(centroid[1]) - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)  # Green color for the text

            # Resize the frame for display
            display_frame = cv2.resize(self.current_frame, (self.display_width, self.display_height))
            display_frame = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)

            # Display the resized annotated frame
            cv2.imshow("Tracker View", display_frame)
            cv2.waitKey(1)

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create the node
    tracker_node = TrackerNode(screen_width=640, screen_height=480, display_width=640, display_height=480)
    
    # Spin the node so the callback functions are called
    rclpy.spin(tracker_node)
    
    # Destroy the node explicitly
    tracker_node.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
