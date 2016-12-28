#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy,roslib
from std_msgs.msg import Bool
from ardrone_autonomy.msg import Navdata
from drone_controller import droneStatus
from ardrone_project.msg import ImageCalc

controlStatus = {
    0:None,
    1:"Take_Off_Unit",
    2:"Land_Unit",
    3:"Follow_controller",
}

class Central_Control_Unit:
    def __init__(self):
        rospy.init_node('central_control_unit',anonymous = False)

        self.pubEnableControl = rospy.Publisher('follow_controller/enable_control', Bool, queue_size=3)
        self.pubEnableTakeOff = rospy.Publisher('take_off_unit/enable_control', Bool, queue_size=3)
        self.pubEnableLand = rospy.Publisher('land_unit/enable_control', Bool, queue_size=3)

        rospy.Subscriber('Central_Control_Unit/enable', Bool, self.callbacEnable)
        rospy.Subscriber('ardrone/navdata', Navdata, self.callbackNavdata)
        rospy.Subscriber('image_converter/calc', ImageCalc, self.callbackCalc)

        self.who_in_control = controlStatus[0]
        self.enable_toggle_control = False
        self.end_visible = False
        self.path_visible = False
        self.img_calc = ImageCalc() #elements in msg definition are assigned zero values by the default constructor.
        self.droneStatus = "Unknown"
        self.altitude = None

    def callbackNavdata(self,navdata):
        self.droneStatus = droneStatus[navdata.state]
        self.altitude = navdata.altd # not clear from documentation if [cm] or [mm]

    def callbacEnable(self,msg):
        enable_flag = msg.data

        if enable_flag: rospy.sleep(1) #if central controll unit is enabled, its action is delayed by 10 seconds
        self.enable_toggle_control = enable_flag

    def callbackCalc(self, data):
        """

        :type data: ImageCalc object
        """
        self.path_visible = data.is_visible
        self.img_calc = data


    def cleanup(self):
        print("central_control_unit cleanup method")

if __name__ == "__main__":
    ccu = Central_Control_Unit()
    try:
        while not rospy.is_shutdown():
            if ccu.enable_toggle_control:
                # toggle control block
                if ccu.droneStatus == "Landed":
                    ccu.pubEnableControl.publish(False)
                    ccu.pubEnableLand.publish(False)
                    ccu.pubEnableTakeOff.publish(True)
                    if ccu.who_in_control is not controlStatus[1]:
                        ccu.who_in_control = controlStatus[1]
                        print("Central Unit: I gave control to Take Off Unit!")
                elif ccu.end_visible:
                    ccu.pubEnableControl.publish(False)
                    ccu.pubEnableLand.publish(True)
                    ccu.pubEnableTakeOff.publish(False)
                    if ccu.who_in_control is not controlStatus[2]:
                        ccu.who_in_control = controlStatus[2]
                        print("Central Unit: I gave control to Land Unit!")
                elif ccu.path_visible and ccu.altitude > 1500 and ccu.img_calc.angle < 10 and ccu.img_calc.distance < 100:
                    #condition to give controol to follow controller
                    #in case drone sees the path and is well stabilized above it
                    ccu.pubEnableControl.publish(True)
                    ccu.pubEnableLand.publish(False)
                    ccu.pubEnableTakeOff.publish(False)
                    if ccu.who_in_control is not controlStatus[3]:
                        ccu.who_in_control = controlStatus[3]
                        print("Central Unit: I gave control to Follow Controller!")
                # end of toggle control block
            else:
                rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        print("central_control_unit: ROSInterruptException")
    finally:
        ccu.cleanup()