#!/usr/bin/env python
import roslib

roslib.load_manifest('ardrone_project')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Bool
from ardrone_project.msg import ImageCalc
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from scipy import spatial
from scipy.cluster.vq import vq, kmeans, whiten
import numpy as np


# ASSUMES BOTTOM CAMERA IS TOGGLED!
# camera channel is 1

class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/ardrone/bottom/image_raw", Image,
                                          self.callback)  # subscribe to drop topic

        self.image_pub = rospy.Publisher("image_converter/img", Image, queue_size=10)  # queue?
        self.image_pub_calc = rospy.Publisher("image_converter/calc", ImageCalc, queue_size=10)  # queue?
        self._is_visible = False

    def callback(self, data):
        img_calc = ImageCalc()
        time_stamp = data.header.stamp
        img_calc.time_stamp = time_stamp

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # self.image_pub.publish(data)
        except CvBridgeError as e:
            print(e)
        except:
            pass

        # filtering noise
        kernel = np.ones((5, 5), np.float32) / 25
        smoothed = cv2.filter2D(cv_image, -1, kernel)

        # find transition between BLUE and RED colours
        hsv = cv2.cvtColor(smoothed, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([110, 80, 20])
        upper_blue = np.array([130, 255, 160])
        mask_b = cv2.inRange(hsv, lower_blue, upper_blue)


        closing_b = cv2.erode(mask_b, np.ones((6, 6), np.uint8), iterations=1)
        dilation_b = cv2.dilate(closing_b, np.ones((20, 20), np.uint8), iterations=1)
        resB = cv2.bitwise_and(cv_image, cv_image, mask=mask_b)

        lower_red = np.array([160, 100, 30])
        upper_red = np.array([179, 255, 150])
        mask_r = cv2.inRange(hsv, lower_red, upper_red)


        closing_r = cv2.erode(mask_r, np.ones((6, 6), np.uint8), iterations=1)
        dilation_r = cv2.dilate(closing_r, np.ones((20, 20), np.uint8), iterations=1)
        resR = cv2.bitwise_and(cv_image, cv_image, mask=mask_r)

        mask_br = cv2.bitwise_and(dilation_r, dilation_b)
        mask_br = cv2.dilate(mask_br, np.ones((40, 40), np.uint8), iterations=1)
        resBR = cv2.bitwise_and(cv_image, cv_image, mask=mask_br)

        # remove unwanted countours from the mask (by enclosing circle radius)
        contours, hierarchy = cv2.findContours(mask_br, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        height, width = cv_image.shape[:2]

        chosen_cnt = None
        for h, cnt in enumerate(contours):
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            minRad = height / 4
            if radius >= minRad:
                chosen_cnt = cnt

        if (chosen_cnt is None):  # path is not visible
            img_calc.is_visible = False
        else:  # path is visible
            mask_final = np.zeros(cv_image.shape, np.uint8)
            cv2.drawContours(mask_final, [chosen_cnt], 0, (255, 0, 0), -1)
            mask_final = mask_final[:, :, 0]
            res1 = cv2.bitwise_and(cv_image, cv_image, mask=mask_final)  # res1 is red/blue band including floor margins

            # cut red/blue line from the band - to determine forward/backward direction
            edges = cv2.Canny(smoothed, 30, 60)
            mask_final_eroded = cv2.erode(mask_final, np.ones((40, 40), np.uint8), iterations=1)
            edges = cv2.bitwise_and(edges, edges, mask=mask_final_eroded)
            line_mask = cv2.dilate(edges, np.ones((25, 25), np.uint8), iterations=1)
            res2 = cv2.bitwise_and(cv_image, cv_image, mask=line_mask)

            thin_line_mask = cv2.erode(line_mask, np.ones((25, 25), np.uint8), iterations=1)

            # calculate Nearest/Centeral point location & direction
            try:
                h, w = line_mask.shape[:2]
                line_yx = np.argwhere(thin_line_mask == 255)
                pt = [h / 2, w / 2]  # <-- the point to find
                _, index = spatial.KDTree(line_yx).query(pt, 50)  # distance always positive
                nearest_pt = line_yx[index]  # <-- the nearest point to center of frame
                nearest_pt, _ = kmeans(nearest_pt, 1)
                nearest_pt = nearest_pt[0]
                y_nearest = nearest_pt[0]
                x_nearest = nearest_pt[1]

                NN_indices_out = spatial.KDTree(line_yx).query_ball_point(nearest_pt, 100)
                NN_indices_in = spatial.KDTree(line_yx).query_ball_point(nearest_pt, 80)
                NN_extreme_indices = list(set(NN_indices_out) - set(NN_indices_in))
                intersection_pixels = line_yx[NN_extreme_indices]
                centroids, _ = kmeans(intersection_pixels, 2)
                center1_x = centroids[0][1]
                center1_y = centroids[0][0]
                center2_x = centroids[1][1]
                center2_y = centroids[1][0]

                if ((center1_x - center2_x) ** 2 + (
                    center1_y - center2_y) ** 2) ** 0.5 < 100: raise "Cannot find two centroids"

                if center2_y > center1_y:
                    center2_y, center1_y = center1_y, center2_y  # swap variables
                    center2_x, center1_x = center1_x, center2_x

                # angle calculated between arrow and the vertical axis (POSITIVE = arrow to the RIGHT)
                # angle between [-90,90]
                angle = -np.angle((center2_x - center1_x) + (center2_y - center1_y) * 1j,
                                  deg=True) - 90  # consider red/blue orientation
                #cv2.line(res1, (center1_x, center1_y), (center2_x, center2_y), (255, 0, 255), 3)

                # calculate nearest point relative to straight line
                v = np.array([center2_y - center1_y, center2_x - center1_x])
                P = np.array([h / 2 - center1_y, w / 2 - center1_x])
                b = float(np.inner(P, v)) / float(np.inner(v, v))
                nearest_pt_on_line = [center1_y, center1_x] + np.around((b * v)).astype(int)
                y_nearest_pt_on_line = nearest_pt_on_line[0]
                x_nearest_pt_on_line = nearest_pt_on_line[1]
                distance = ((x_nearest_pt_on_line - w / 2) ** 2 + (y_nearest_pt_on_line - h / 2) ** 2) ** 0.5

                # dH = 60
                # CALCULATION METHOD 1
                # x_mid_line = int(np.average(np.argwhere(line_mask[h / 2, :] == 255)))  ## run time error when line doesn't cross the central row in the frame
                # x_mid_line_1 = int(np.average(np.argwhere(line_mask[h / 2 + dH / 2, :] == 255)))
                # x_mid_line_2 = int(np.average(np.argwhere(line_mask[h / 2 - dH / 2, :] == 255)))
                # calculate distance & angle
                # horiz_dist_px = w / 2 - x_mid_line  # POSITIVE distance = camera is RIGHT to target
                # angle = np.angle(x_mid_line_2 - x_mid_line_1 + dH * 1j, deg=True) - 90  # consider red/blue orientation
                # cv2.circle(res1, (x_mid_line, h / 2), 5, (0, 255, 0), -1)
                # cv2.line(res1, (x_mid_line_1, h / 2 + dH / 2), (x_mid_line_2, h / 2 - dH / 2), (255, 0, 0), 3)

                # CALCULATION METHOD 2
                # x_nearest_1 = int(np.average(np.argwhere(line_mask[y_nearest + dH / 2, :] == 255)))
                # x_nearest_2 = int(np.average(np.argwhere(line_mask[y_nearest - dH / 2, :] == 255)))
                # cv2.line(res1, (x_nearest_1, y_nearest + dH / 2), (x_nearest_2, y_nearest - dH / 2), (255, 0, 255), 3)
                # angle = np.angle(x_nearest_2 - x_nearest_1 + dH * 1j, deg=True) - 90  # consider red/blue orientation

                cv2.circle(thin_line_mask, (w / 2, h / 2), 10, (0, 0, 255), -1)
                # cv2.circle(res1, (x_nearest, y_nearest), 5, (0, 255, 0), -1)
                #cv2.circle(res1, (x_nearest_pt_on_line, y_nearest_pt_on_line), 5, (0, 255, 255), -1)
                # cv2.circle(res1, (center2_x, center2_y), 5, (0, 255, 0), -1)

                #cv2.line(res1, (x_nearest_pt_on_line, y_nearest_pt_on_line), (w / 2, h / 2), (255, 0, 0), 3)

                # cv2.putText(res1, 'shift: {}'.format(horiz_dist_px), (w / 2, 100), cv2.FONT_ITALIC, 1, (255, 255, 255), 2)
                #cv2.putText(res1, 'distance: {}'.format(distance), (w / 2, 100), cv2.FONT_ITALIC, 1, (255, 255, 255), 2)
                #cv2.putText(res1, 'angle: {0:.3f}'.format(angle), (w / 2, 50), cv2.FONT_ITALIC, 1, (255, 255, 255), 2)
                # cv2.putText(res1, 'secs: {}'.format(time_stamp.secs), (w / 2, 150), cv2.FONT_ITALIC, 0.5, (255, 255, 255), 1)
                # cv2.putText(res1, 'nsecs: {}'.format(time_stamp.nsecs), (w / 2, 180), cv2.FONT_ITALIC, 0.5, (255, 255, 255), 1)

                #cv2.imshow('res1', thin_line_mask)
                #cv2.imshow('res2', res2)
                cv2.imshow('thin', resR)
                #cv2.imshow('frame', cv_image)
                cv2.waitKey(1)

                img_calc.is_visible = True
                # msg_calc.shift = horiz_dist_px
                img_calc.distance = distance

                ################################################################################
                img_calc.arrow_x = x_nearest_pt_on_line - w / 2  # x grows from left to right  ###x_nearest
                img_calc.arrow_y = y_nearest_pt_on_line - h / 2  # y grows from top to bottom  ###y_nearest
                img_calc.angle = angle

            except:
                img_calc.is_visible = False

        if (self._is_visible is not img_calc.is_visible):
            print("image_converter: Visibile = {}".format(img_calc.is_visible))  # print only when visibility changes
        self._is_visible = img_calc.is_visible

        try:
            self.image_pub_calc.publish(img_calc)
            # self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except:  # message can arrive before node was initialized
            pass


def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
