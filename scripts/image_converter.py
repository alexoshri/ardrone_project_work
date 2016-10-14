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
from scipy.ndimage.interpolation import shift
from scipy.cluster.vq import vq, kmeans, whiten
import numpy as np


# ASSUMES BOTTOM CAMERA IS TOGGLED!
# ASSUMES RED ON RIGHT WHEN STARTED
# TODO: write initialization process to determine initial orientation
# camera channel is 1

class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/ardrone/bottom/image_raw_drop", Image, self.callback)  # subscribe to drop topic

        self.image_pub = rospy.Publisher("image_converter/img", Image, queue_size=10)  # queue?
        self.image_pub_calc = rospy.Publisher("image_converter/calc", ImageCalc, queue_size=10)  # queue?
        self._is_visible = False
        #self._orientation_forward = True#np.ones((1,10), dtype=bool) #FORWARD direction if RED is on the RIGHT, initiation assumes forward direction

    def callback(self, data):
        img_calc = ImageCalc()
        time_stamp = data.header.stamp
        img_calc.time_stamp = time_stamp

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #self.image_pub.publish(data)
        except CvBridgeError as e:
            print(e)
        except:
            pass

        # filtering noise
        kernel = np.ones((5, 5), np.float32) / 25
        smoothed = cv2.filter2D(cv_image, -1, kernel)

        # find transition between BLUE and RED colours
        hsv = cv2.cvtColor(smoothed, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 80, 0])
        upper_blue = np.array([130, 255, 100])
        mask_b = cv2.inRange(hsv, lower_blue, upper_blue)

        closing_b = cv2.erode(mask_b, np.ones((6, 6), np.uint8), iterations=1)
        dilation_b = cv2.dilate(closing_b, np.ones((20, 20), np.uint8), iterations=1)

        lower_red_1 = np.array([160, 150, 0])
        upper_red_1 = np.array([179, 255, 150])
        mask_r_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)

        lower_red_2 = np.array([0, 150, 0])
        upper_red_2 = np.array([10, 255, 150])
        mask_r_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)

        mask_r = np.logical_or(mask_r_1,mask_r_2)
        mask_r = 255 * mask_r.astype('uint8')

        closing_r = cv2.erode(mask_r, np.ones((6, 6), np.uint8), iterations=1)
        dilation_r = cv2.dilate(closing_r, np.ones((20, 20), np.uint8), iterations=1)

        mask_br = cv2.bitwise_and(dilation_r, dilation_b)
        mask_br = cv2.dilate(mask_br, np.ones((40, 40), np.uint8), iterations=1)

        # remove unwanted countours from the mask (by enclosing circle radius)
        contours, hierarchy = cv2.findContours(mask_br, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        height, width = cv_image.shape[:2]

        chosen_cnt = None
        for h, cnt in enumerate(contours):
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            minRad = height/10
            if radius >= minRad:
                chosen_cnt = cnt


        if (chosen_cnt is None): # path is not visible
            img_calc.is_visible = False
        else: # path is visible
            mask_final = np.zeros(cv_image.shape, np.uint8)
            cv2.drawContours(mask_final, [chosen_cnt], 0, (255, 0, 0), -1)
            mask_final = mask_final[:, :, 0]
            res1 = cv2.bitwise_and(cv_image, cv_image, mask=mask_final) # res1 is red/blue band including floor margins

            # cut red/blue mask from the band - to determine forward/backward direction
            edges = cv2.Canny(smoothed, 30, 60)
            mask_final_eroded = cv2.erode(mask_final, np.ones((40, 40), np.uint8), iterations=1)
            edges = cv2.bitwise_and(edges, edges, mask=mask_final_eroded)
            direction_mask = cv2.dilate(edges, np.ones((25, 25), np.uint8), iterations=1)

            thin_line_mask = cv2.erode(direction_mask, np.ones((25, 25), np.uint8), iterations=1)

            # calculate target point location & direction
            # NOTE - frame coordinates: x grows from left to right, y grows from top to bottom
            try:
                h, w = cv_image.shape[:2]
                line_yx = np.argwhere(thin_line_mask == 255)
                pt = [h/2, w/2] # <-- the point to find
                _, index = spatial.KDTree(line_yx).query(pt,50)
                nearest_pt = line_yx[index] # <-- the NEAREST POINT to center of frame
                nearest_pt, _ = kmeans(nearest_pt, 1)
                nearest_pt = nearest_pt[0]

                NN_indices_out = spatial.KDTree(line_yx).query_ball_point(nearest_pt, 100)
                NN_indices_in = spatial.KDTree(line_yx).query_ball_point(nearest_pt, 80)
                NN_extreme_indices = list(set(NN_indices_out) - set(NN_indices_in))
                intersection_pixels = line_yx[NN_extreme_indices]
                centroids, _ = kmeans(intersection_pixels, 2)
                center1_x = centroids[0][1]
                center1_y = centroids[0][0]
                center2_x = centroids[1][1]
                center2_y = centroids[1][0]

                if ((center1_x - center2_x)**2 + (center1_y - center2_y)**2)**0.5 < 100: raise "Cannot find two centroids"

                # This conditional swap insures the "arrow" between centroids is defined between [ -90,90 )
                if center2_y > center1_y:
                    center2_y, center1_y = center1_y, center2_y #swap variables
                    center2_x, center1_x = center1_x, center2_x
                if center2_y == center1_y and center1_x > center2_x:
                    center2_y, center1_y = center1_y, center2_y #swap variables
                    center2_x, center1_x = center1_x, center2_x

                # CALCULATION of nearest point relative to straight line
                v = np.array([center2_y - center1_y, center2_x - center1_x])
                P = np.array([h/2 - center1_y,w/2 - center1_x])
                b = float(np.inner(P,v))/float(np.inner(v, v))
                nearest_pt_on_line = [center1_y,center1_x] + np.around((b * v)).astype(int)
                y_nearest_pt_on_line = nearest_pt_on_line[0]
                x_nearest_pt_on_line = nearest_pt_on_line[1]

                # ANGLE CALCULATION
                angle = -np.angle((center2_x - center1_x) + (center2_y - center1_y) * 1j, deg=True) - 90    # angle calculated between arrow and the vertical axis (POSITIVE = arrow to the LEFT)
                angle_perp = angle - 90
                perp = -np.array([np.cos(angle_perp * np.pi /180), np.sin(angle_perp * np.pi /180)])

                dX = 50
                dY = 50
                pt_check_color = np.around(np.array([dY + 80 * perp[0],dX + 80 * perp[1]])).astype(int) # far away point on the right perpndicular
                mask_b_cropped = mask_b[y_nearest_pt_on_line - dY : y_nearest_pt_on_line + dY, x_nearest_pt_on_line - dX : x_nearest_pt_on_line + dX]
                mask_r_cropped = mask_r[y_nearest_pt_on_line - dY : y_nearest_pt_on_line + dY, x_nearest_pt_on_line - dX : x_nearest_pt_on_line + dX]
                mask_br_cropped = np.logical_or(mask_b_cropped,mask_r_cropped)
                direction_mask = direction_mask[y_nearest_pt_on_line - dY : y_nearest_pt_on_line + dY, x_nearest_pt_on_line - dX : x_nearest_pt_on_line + dX]
                #croped_frame = cv_image[y_nearest_pt_on_line - dY: y_nearest_pt_on_line + dY,x_nearest_pt_on_line - dX: x_nearest_pt_on_line + dX]
                croped_hsv = hsv[y_nearest_pt_on_line - dY: y_nearest_pt_on_line + dY,x_nearest_pt_on_line - dX: x_nearest_pt_on_line + dX]
                direction_mask = 255 * np.logical_or(direction_mask,mask_br_cropped).astype('uint8')
                direction_mask_yx = np.argwhere(direction_mask == 255)
                _, index = spatial.KDTree(direction_mask_yx).query(pt_check_color,200)
                nearest_pt_on_direction = direction_mask_yx[index]

                hsv_nearest_pt_on_direction = croped_hsv[nearest_pt_on_direction[:,0], nearest_pt_on_direction[:,1],:]
                in_range_red_1 = np.all(np.logical_and(hsv_nearest_pt_on_direction >= lower_red_1, hsv_nearest_pt_on_direction <= upper_red_1),axis = 1)
                in_range_red_2 = np.all(np.logical_and(hsv_nearest_pt_on_direction >= lower_red_2, hsv_nearest_pt_on_direction <= upper_red_2),axis = 1)
                num_red = np.count_nonzero(np.logical_or(in_range_red_1,in_range_red_2))
                if num_red > 50:
                    is_red = True
                else:
                    is_red = False

                #TODO: add buffer
                #if not is_red and self._orientation_forward == True and abs(abs(angle) - 90) < 10: self._orientation_forward = False
                #if is_red and self._orientation_forward == False and abs(abs(angle) - 90) < 10: self._orientation_forward = True

                #if self._orientation_forward == False: angle = angle + 180
                if angle > 180: angle -= 360

                img_calc.is_visible = True
                img_calc.arrow_x = x_nearest_pt_on_line - w / 2
                img_calc.arrow_y = y_nearest_pt_on_line - h / 2
                img_calc.arrow_x_forward = center2_x - x_nearest_pt_on_line
                img_calc.arrow_y_forward = center2_y - y_nearest_pt_on_line
                img_calc.distance = ((x_nearest_pt_on_line - w / 2) ** 2 + (y_nearest_pt_on_line - h / 2) ** 2) ** 0.5
                img_calc.angle = angle

                # Visualization
                #res2 = cv2.bitwise_and(croped_frame, croped_frame, mask=direction_mask)
                #res2[nearest_pt_on_direction[:, 0], nearest_pt_on_direction[:, 1], :] = [255, 255, 255]
                #cv2.circle(res2, (pt_check_color[1], pt_check_color[0]), 5, (255, 255, 255), -1)
                #cv2.line(res2, (dX, dY), (pt_check_color[1], pt_check_color[0]), (255, 0, 0), 3)

                cv2.circle(res1, (w / 2, h / 2), 5, (0, 0, 255), -1)
                #cv2.circle(res1, (x_nearest, y_nearest), 5, (0, 255, 0), -1)
                cv2.circle(res1, (x_nearest_pt_on_line, y_nearest_pt_on_line), 5, (0, 255, 255), -1)
                #cv2.circle(res1, (center2_x, center2_y), 5, (0, 255, 0), -1)
		
                cv2.line(res1, (x_nearest_pt_on_line, y_nearest_pt_on_line), (w/2, h/2), (255, 0, 0), 3)
                cv2.line(res1, (center1_x, center1_y), (center2_x, center2_y), (255, 0, 255), 3)

                cv2.putText(res1, 'distance: {}'.format(img_calc.distance), (w / 2, 100), cv2.FONT_ITALIC, 1, (255, 255, 255), 2)
                cv2.putText(res1, 'angle: {0:.3f}'.format(img_calc.angle), (w / 2, 50), cv2.FONT_ITALIC, 1, (255, 255, 255), 2)
                #cv2.putText(res1, 'secs: {}'.format(time_stamp.secs), (w / 2, 150), cv2.FONT_ITALIC, 0.5, (255, 255, 255), 1)
                #cv2.putText(res1, 'nsecs: {}'.format(time_stamp.nsecs), (w / 2, 180), cv2.FONT_ITALIC, 0.5, (255, 255, 255), 1)
                cv2.putText(res1, '#red points: {}'.format(num_red), (w / 2, 150), cv2.FONT_ITALIC, 1, (255, 255, 255), 2)
                #cv2.putText(res1, '#is forward: {}'.format(self._orientation_forward), (w / 2, 200), cv2.FONT_ITALIC, 1, (255, 255, 255),2)

                cv2.imshow('res1', res1)
                #cv2.imshow('res2',res2)
                #cv2.imshow('thin',thin_line_mask)
                #cv2.imshow('frame',cv_image)
                cv2.waitKey(1)

            except:
                img_calc.is_visible = False

        if (self._is_visible is not img_calc.is_visible):
            print("image_converter: Visibile = {}".format(img_calc.is_visible)) #print only when visibility changes
        self._is_visible = img_calc.is_visible

        try:
            self.image_pub_calc.publish(img_calc)
            #self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except: # message can arrive before node was initialized
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
