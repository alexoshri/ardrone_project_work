#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, roslib
from std_msgs.msg import String, Bool, Empty
from ardrone_autonomy.msg import Navdata
from drone_controller import droneStatus

class takeOffUnit:
    def __init__(self):
        rospy.init_node('take_off_unit', anonymous=True)
        self.pubCommand = rospy.Publisher('drone_controller/com', String, queue_size=10)
        rospy.Subscriber('take_off_unit/enable_control', Bool, self.callbackEnableControl)
        rospy.Subscriber('ardrone/navdata', Navdata, self.callbackNavdata)
        self.enableControl = False
        self._droneStatus = "Unknown"

    def callbackEnableControl(self,msg):
        enable_flag = msg.data
        if not self.enableControl and enable_flag:
            self.pubCommand.publish("TAKEOFF")
            while self._droneStatus is not "Hovering":
                rospy.sleep(0.2)

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
                pass
            else:
                rospy.sleep(0.5)
    except rospy.ROSInterruptException:
        print("take_off_unit: ROSInterruptException")
    finally:
        tou.cleanup()
