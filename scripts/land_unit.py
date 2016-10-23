#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, roslib
from std_msgs.msg import Bool, String

class LandUnit:
    def __init__(self):
        rospy.init_node('land_unit', anonymous=True)
        self.pubCommand = rospy.Publisher('drone_controller/com', String, queue_size=10)
        rospy.Subscriber('land_unit/enable_control', Bool, self.callbackEnableControl)

    def callbackEnableControl(self,msg):
        enable_flag = msg.data
        if enable_flag:
            self.pubCommand.publish("LAND")

    def cleanup(self):
        print("LandUnit: cleanup method")

if __name__ == "__main__":
    lu = LandUnit()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("LandUnit: ROSInterruptException")
    finally:
        lu.cleanup()
