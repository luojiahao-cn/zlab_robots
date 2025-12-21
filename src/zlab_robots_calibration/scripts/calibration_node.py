#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CalibrationNode:
    def __init__(self):
        rospy.init_node('zlab_calibration_node')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        rospy.loginfo('Calibration node started, waiting for images...')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr('CV Bridge error: {}'.format(e))
            return
        # TODO: 调用apriltag检测与标定逻辑
        cv2.imshow('D405 Image', cv_image)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    node = CalibrationNode()
    node.run()
