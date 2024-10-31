import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera')
        
        # Publisher to camera/image_raw
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        
        # Define OpenCV video capture and loop mechanism
        self.cap = cv2.VideoCapture('/home/sergio/ros2_ws/src/yolo_pkg/feed/WIN_20241031_04_28_25_Pro.mp4')
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open video file")
            rclpy.shutdown()

        # CvBridge to convert OpenCV images to ROS Image messages
        self.bridge = CvBridge()
        
        # Timer callback to publish frames
        timer_period = 1.0 / 30  # 30 FPS
        self.timer = self.create_timer(timer_period, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        
        # Loop the video if it ends
        if not ret:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.cap.read()
        
        # Convert and publish the frame
        if ret:
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(ros_image)
        else:
            self.get_logger().warning("Failed to read frame from video")

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)

    # Clean up when done
    camera_node.cap.release()
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
