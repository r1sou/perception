import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

topic = '/image_combine_raw'

class Display(Node):
    def __init__(self):
        super().__init__('display')
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def listener_callback(self, data):
        cv2.imshow("camera", self.bridge.imgmsg_to_cv2(data, "bgr8"))
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    display = Display()
    rclpy.spin(display)
    display.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
