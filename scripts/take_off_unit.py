#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, roslib
from std_msgs.msg import String, Bool, Empty
from ardrone_autonomy.msg import Navdata
from drone_controller import droneStatus
from ardrone_project.msg import ImageCalc

class takeOffUnit:
    def __init__(self):
        rospy.init_node('take_off_unit', anonymous=True)
        self.pubCommand = rospy.Publisher('drone_controller/com', String, queue_size=10)
        rospy.Subscriber('take_off_unit/enable_control', Bool, self.callbackEnableControl)
        rospy.Subscriber('ardrone/navdata', Navdata, self.callbackNavdata)
        rospy.Subscriber('image_converter/calc', ImageCalc, self.callbackCalc)

        self.img_calc = ImageCalc() #elements in msg definition are assigned zero values by the default constructor.
        self._rateAng = rospy.Rate(1.5)
        self._rateHoriz = rospy.Rate(1.8)
        self._rateHover = rospy.Rate(1.5)  # 5Hz


        self.is_visible = False
        self.enableControl = False
        self._droneStatus = "Unknown"
        self.UPWARD_RATIO = 3
        self.upward_counter = 1

    def callbackCalc(self, data):
        """

        :type data: ImageCalc object
        """
        self.is_visible = data.is_visible
        if self.is_visible:
            self.img_calc = data
        else:
            self.upward_counter = 1 # count reset


    def callbackEnableControl(self,msg):
        enable_flag = msg.data
        if not self.enableControl and enable_flag:
            self.pubCommand.publish("TAKEOFF")
            while self._droneStatus is not "Hovering":
                rospy.sleep(0.3)


            # takeoff complete - drone entered hovering state
            # TODO: set altitude max to a value greater than desired height (using rosparam)

        self.enableControl = enable_flag

    def callbackNavdata(self,navdata):
        self._droneStatus = droneStatus[navdata.state]

    def cleanup(self):
        print("takeOffUnit: cleanup method")

if __name__ == "__main__":
    tou = takeOffUnit()
    try:
        while not rospy.is_shutdown():
            if tou.enableControl:
                # elevate drone to desired height without loosing visibility
                # use stabilization controller
                ### PLANAR CONTROL BLOCK
                if tou.is_visible:
                    if tou.img_calc.distance <= 100:
                        x_vel = -float(tou.img_calc.arrow_x) * 0.01
                        y_vel = -float(tou.img_calc.arrow_y) * 0.01
                    else:
                        x_vel = -float(tou.img_calc.arrow_x) * 0.005
                        y_vel = -float(tou.img_calc.arrow_y) * 0.005

                    if abs(x_vel) > 1 or abs(y_vel) > 1:
                        norm = (x_vel ** 2 + y_vel ** 2) ** 0.5
                        x_vel = x_vel / norm
                        y_vel = y_vel / norm
                    command = "SET_VELOCITY {} {} 0 0 0 0".format(y_vel, x_vel)
                    tou.pubCommand.publish(command)
                    tou._rateHoriz.sleep()
                    if tou.img_calc.distance > 100:
                        command = "SET_VELOCITY {} {} 0 0 0 0".format(y_vel, x_vel) #publish same command withous bias
                        tou.pubCommand.publish(command)
                        dt = 0.0015 * tou.img_calc.distance
                        rospy.sleep(dt)  # sleep seconds

                    command = "HOVER"
                    tou.pubCommand.publish(command)
                    tou._rateHover.sleep()
                ### END OF PLANAR CONTROL BLOCK

                ### FLIGHT UPWARD executed once in controller.FORWARD_RATIO iterations
                if tou.upward_counter == 0 and tou.is_visible:
                    tou.pubCommand.publish(command)
                    rospy.sleep(0.3)

                    command = "HOVER"
                    tou.pubCommand.publish(command)
                    tou._rateHover.sleep()
                ### END OF FLIGHT UPWARD BLOCK

                tou.upward_counter += 1
                tou.upward_counter = tou.upward_counter % tou.UPWARD_RATIO

                ### ANGULAR CONTROL BLOCK
                if tou.is_visible:
                    if abs(tou.img_calc.angle) <= 10:
                        angular_vel = 0.03 * tou.img_calc.angle
                    else:
                        angular_vel = 0.01 * tou.img_calc.angle
                    if angular_vel > 0.3: angular_vel = 0.3
                    if angular_vel < -0.3: angular_vel = -0.3
                    command = "SET_VELOCITY {} 0 0 0 0 {}".format(0, angular_vel)
                    tou.pubCommand.publish(command)
                    tou._rateAng.sleep()
                    if abs(tou.img_calc.angle) > 10:
                        command = "SET_VELOCITY {} 0 0 0 0 {}".format(0, angular_vel) #publish same command withous bias
                        tou.pubCommand.publish(command)
                        dt = 0.007 * abs(tou.img_calc.angle)
                        rospy.sleep(dt)  # sleep seconds

                    command = "HOVER"
                    tou.pubCommand.publish(command)
                    tou._rateHover.sleep()
                ### END OF ANGULAR CONTROL BLOCK
            else:
                rospy.sleep(0.5)
    except rospy.ROSInterruptException:
        print("take_off_unit: ROSInterruptException")
    finally:
        tou.cleanup()
