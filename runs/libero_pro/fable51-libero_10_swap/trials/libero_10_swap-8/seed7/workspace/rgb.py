import sys, numpy as np, rclpy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cloud import _grab
rclpy.init(); node = rclpy.create_node("rgb_grab")
for cam in sys.argv[1:]:
    m = _grab(node, f"/{cam}/color/image_raw", Image)
    img = CvBridge().imgmsg_to_cv2(m, desired_encoding="bgr8")
    cv2.imwrite(f"snaps/final_{cam}.png", img); print("saved", cam, img.shape)
