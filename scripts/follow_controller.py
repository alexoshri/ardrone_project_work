#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy,roslib
from std_msgs.msg import String, Bool, Empty
from ardrone_autonomy.msg import Navdata
from drone_controller import droneStatus
from ardrone_project.msg import ImageCalc

class follow_controller:
    def __init__(self):
        self.img_calc = ImageCalc()
        self.enableControl = False

        rospy.init_node('follow_controller',anonymous = False)
        self._pubCommand = rospy.Publisher('drone_controller/com', String ,queue_size = 20)
        self._rateCommand = rospy.Rate(2.0)
        self._rateHoriz = rospy.Rate(1.8)
        self._rateHover = rospy.Rate(2.6)  # 5Hz
        self._rateForward = rospy.Rate(1.0)

        rospy.Subscriber('image_converter/calc', ImageCalc, self.callbackCalc)
        rospy.Subscriber('ardrone/navdata', Navdata, self.callbackNavdata)
        rospy.Subscriber('follow_controller/enable_control', Bool, self.callbackEnableControl)        

    def callbackEnableControl(self, msg):
        flag = msg.data
        print("Enable/Disable Control = {}".format(flag) + "\n")
        self.enableControl = flag
	

    def callbackNavdata(self,navdata):        
        self._droneStatus = navdata.state
    
    def callbackCalc(self,data):
        """

        :type data: ImageCalc object
        """
        self.img_calc = data
        if self.enableControl == True and self.img_calc.is_visible == False: #if control enables and path is not visible
            command = "LAND"
            #print("follow_cntroller: " + command + "\n")
            controller._pubCommand.publish(command)



    def cleanup(self):
        print("followController cleanup method")

    def sleep(self):
        self._rateCommand.sleep()


if __name__ == "__main__":
    controller = follow_controller()
    try:
        while not rospy.is_shutdown():
            if (controller.enableControl) and (controller.img_calc.is_visible): # if control enabled and path is visible
                #horiz_vel = (1/1000) * controller.shift # POSITIVE velocity = move LEFT
                x_vel = -float(controller.img_calc.arrow_x)*0.014
                y_vel = -float(controller.img_calc.arrow_y)*0.014
                if abs(x_vel) > 1 or abs(y_vel):
                    norm = (x_vel**2 + y_vel**2)**0.5
                    x_vel = x_vel/norm
                    y_vel = y_vel/norm
                command = "SET_VELOCITY {} {} 0 0 0 0".format(x_vel, y_vel)
                #print("follow_cntroller: " + command + " frame_time_stamp: {} {}".format(controller.img_calc.time_stamp.secs, controller.img_calc.time_stamp.nsecs) + "\n")
                controller._pubCommand.publish(command)
                controller._rateHoriz.sleep()

                command = "HOVER"
                #print("follow_cntroller: " + command + " frame_time_stamp: {} {}".format(controller.img_calc.time_stamp.secs, controller.img_calc.time_stamp.nsecs) + "\n")
                controller._pubCommand.publish(command)
                controller._rateHover.sleep()

                angular_vel = 0.07 * controller.img_calc.angle
                if angular_vel > 1: angular_vel = 1.0
                if angular_vel < -1: angular_vel = -1.0
                command = "SET_VELOCITY 0 0 0 0 0 {}".format(angular_vel)
                #print("follow_cntroller: " + command + " frame_time_stamp: {} {}".format(controller.img_calc.time_stamp.secs, controller.img_calc.time_stamp.nsecs) + "\n")
                controller._pubCommand.publish(command)
                controller.sleep()

                command = "HOVER"
                #print("follow_cntroller: " + command + " frame_time_stamp: {} {}".format(controller.img_calc.time_stamp.secs, controller.img_calc.time_stamp.nsecs) + "\n")
                controller._pubCommand.publish(command)
                controller._rateHover.sleep()

                if controller.img_calc.distance < 150 and controller.img_calc.angle < 5:
		    print "MOVING FORWARD!"
                    command = "SET_VELOCITY 1.0 0 0 0 0 0"
                    controller._pubCommand.publish(command)
                    controller._rateForward.sleep()

    except rospy.ROSInterruptException:
        print("droneController: ROSInterruptException")
    finally:
        controller.cleanup()

