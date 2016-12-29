#!/usr/bin/env python
import roslib

roslib.load_manifest('ardrone_project')
import sys
import rospy
import cv2
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# ASSUMES BOTTOM CAMERA IS TOGGLED!

class end_detection:
    def __init__(self):
        rospy.init_node('end_detection', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/ardrone/bottom/image_raw_drop", Image, self.callback)  # subscribe to drop topic

        self.is_end_pub = rospy.Publisher("end_detection/is_end", Bool, queue_size=10)  # queue?

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        h , w = cv_image.shape[:2]

        # filtering noise
        blur = cv2.blur(cv_image, (10, 10))

        #yellow mask
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([15, 100, 50])
        upper_yellow = np.array([45, 255, 255])
        mask_y = cv2.inRange(hsv, lower_yellow, upper_yellow)

        closing_y = cv2.erode(mask_y, np.ones((30, 30), np.uint8), iterations=1)
        dilation_y = cv2.dilate(closing_y, np.ones((10, 10), np.uint8), iterations=1)

        num_yellow = np.count_nonzero(dilation_y)
        is_end = num_yellow > 3000
        self.is_end_pub.publish(is_end)

        #Visualization
        cv2.putText(dilation_y, 'num yellow: {}'.format(num_yellow), (w / 2, 100), cv2.FONT_ITALIC, 1, (255, 255, 255),2)
        cv2.putText(dilation_y, 'End Detection = {}'.format(is_end), (int(w / 4), 200), cv2.FONT_ITALIC, 1, (255, 255, 255),2)
        cv2.imshow('frame',dilation_y)
        cv2.imshow('fre',cv_image)
        cv2.waitKey(1)

def main(args):
    ed = end_detection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    except rospy.ROSInterruptException:
        print("take_off_unit: ROSInterruptException")
    finally:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
