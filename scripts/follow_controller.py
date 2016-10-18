#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, roslib
from std_msgs.msg import String, Bool, Empty
from ardrone_autonomy.msg import Navdata
from drone_controller import droneStatus
from ardrone_project.msg import ImageCalc


class follow_controller:
    def __init__(self):
        self.img_calc = ImageCalc()
        self.enableControl = False

        rospy.init_node('follow_controller', anonymous=False)
        self._pubCommand = rospy.Publisher('drone_controller/com', String, queue_size=10)
        self._rateCommand = rospy.Rate(1.5)
        self._rateHoriz = rospy.Rate(1.8)
        self._rateHover = rospy.Rate(1.5)  # 5Hz

        rospy.Subscriber('image_converter/calc', ImageCalc, self.callbackCalc)
        rospy.Subscriber('follow_controller/enable_control', Bool, self.callbackEnableControl)
        self.is_visible = False
        self.FORWARD_RATIO = 4

    def callbackEnableControl(self, msg):
        flag = msg.data
        print("Enable/Disable Control = {}".format(flag) + "\n")
        self.enableControl = flag

    def callbackCalc(self, data):
        """

        :type data: ImageCalc object
        """
        self.is_visible = data.is_visible
        if self.is_visible:
            self.img_calc = data

    def cleanup(self):
        print("followController cleanup method")

    def sleep(self):
        self._rateCommand.sleep()


if __name__ == "__main__":
    controller = follow_controller()
    Bias = 0.0
    forward_conter = 1
    try:
        while not rospy.is_shutdown():
            if controller.enableControl == True:
                if controller.is_visible:
                    if controller.img_calc.turn_indicator_distance < 100:
                        ### PLANAR CONTROL BLOCK
                        x_vel = -float(controller.img_calc.arrow_x) * 0.007
                        y_vel = -float(controller.img_calc.arrow_y) * 0.007
                        if abs(x_vel) > 1 or abs(y_vel) > 1:
                            norm = (x_vel ** 2 + y_vel ** 2) ** 0.5
                            x_vel = x_vel / norm
                            y_vel = y_vel / norm
                        command = "SET_VELOCITY {} {} 0 0 0 0".format(y_vel + Bias, x_vel)
                        controller._pubCommand.publish(command)
                        controller._rateHoriz.sleep()
                        if controller.img_calc.distance > 30:
                            dt = 0.0015 * controller.img_calc.distance
                            rospy.sleep(dt)  # sleep seconds

                        command = "HOVER"
                        controller._pubCommand.publish(command)
                        controller._rateHover.sleep()
                        ### END OF PLANAR CONTROL BLOCK

                    ### FLIGHT FORWARD executed once in controller.FORWARD_RATIO iterations
                    if forward_conter == 0:
                        norm = (controller.img_calc.arrow_x_forward ** 2 + controller.img_calc.arrow_y_forward ** 2) ** 0.5
                        if controller.img_calc.turn_indicator_distance > 5: vel = 0.1
                        else: vel = 0.18
                        x_vel = -float(controller.img_calc.arrow_x_forward) / float(norm) * vel
                        y_vel = -float(controller.img_calc.arrow_y_forward) / float(norm) * vel
                        command = "SET_VELOCITY {} {} 0 0 0 0".format(y_vel, x_vel)
                        controller._pubCommand.publish(command)
                        rospy.sleep(0.7)

                        command = "HOVER"
                        controller._pubCommand.publish(command)
                        controller._rateHover.sleep()
                    ### END OF FLIGHT FORWARD BLOCK

                    forward_conter += 1
                    forward_conter = forward_conter % controller.FORWARD_RATIO

                    angular_vel = 0.07 * controller.img_calc.angle
                    if angular_vel > 1: angular_vel = 1.0
                    if angular_vel < -1: angular_vel = -1.0
                    command = "SET_VELOCITY {} 0 0 0 0 {}".format(Bias, angular_vel)
                    controller._pubCommand.publish(command)
                    controller.sleep()
                    if abs(controller.img_calc.angle) > 5:
                        dt = 0.007 * abs(controller.img_calc.angle)
                        rospy.sleep(dt)  # sleep seconds

                    command = "HOVER"
                    controller._pubCommand.publish(command)
                    controller._rateHover.sleep()
                else:
                    rospy.sleep(0.2)
            else:
                rospy.sleep(0.2)

    except rospy.ROSInterruptException:
        print("droneController: ROSInterruptException")
    finally:
        controller.cleanup()
