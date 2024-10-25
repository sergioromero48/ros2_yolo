# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from ultralytics import YOLO # YOLO model for object detection
import os
 
class ImageSubscriber(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('yolo_test')
          
        # Create the subscriber. This subscriber will receive an Image
        # from the /camera/image_raw topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image, 
            '/camera/image_raw',  # Updated topic
            self.listener_callback, 
            10)
        self.subscription # prevent unused variable warning
          
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        
        # Initialize the YOLO model
        model_path = os.path.join(
            os.path.dirname(__file__),  # This gets the path to the current file (yolo_pkg/yolo_pkg)
            '..',                       # Go one level up (to yolo_pkg)
            'yolo_models',              # Then access the yolo_models directory
            'yolo11n.pt'                # Model file
        )

        # Check if the model file exists
        if not os.path.isfile(model_path):
            self.get_logger().error(f"Model file not found at {model_path}")
        else:
            self.model = YOLO(model_path)
       
    def listener_callback(self, data):
        """
        Callback function.
        """
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        current_frame = cv2.flip(current_frame, 1)
        
        # Convert BGR to RGB for YOLO compatibility
        current_frame_rgb = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        
        # Run YOLO on the image
        results = self.model(current_frame_rgb)
        
        # Display YOLO predictions on the image (bounding boxes, etc.)
        result_image = results[0].plot()
        
        # Convert back to BGR for OpenCV display
        result_image_bgr = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)

        # Fix the window size error
        cv2.namedWindow("YOLO Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("YOLO Detection", 800, 600)  # Set the window size (width, height)

        # Display the result
        cv2.imshow("YOLO Detection", result_image_bgr)
        cv2.waitKey(1)
  
def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create the node
    image_subscriber = ImageSubscriber()
    
    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)
    
    # Destroy the node explicitly
    image_subscriber.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
