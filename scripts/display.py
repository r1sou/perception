import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

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

    def listener_callback(self, msg):
        encoding = msg.encoding
        if encoding == "bgr8":
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            left_image = image[:image.shape[0]//2, :]
            right_image = image[image.shape[0]//2:, :]

            cv2.imshow("left_camera", left_image)
            cv2.imshow("right_camera", right_image)
        elif encoding == "nv12":
            height = msg.height
            width  = msg.width
            nv12 = np.frombuffer(msg.data, np.uint8).reshape((height * 3 // 2, width))
            image = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
            left_image  = image[:height//2, :]
            right_image = image[height//2:, :]
            
            cv2.imshow("left_camera", left_image)
            cv2.imshow("right_camera", right_image)
        
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
