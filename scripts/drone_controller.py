#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy,roslib
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata

droneStatus = {
    0:"Unknown", 
    1:"Init",
    2:"Landed",
    3:"Flying",
    4:"Hovering",
    5:"Test",
    6:"Taking_off",
    7:"Goto_fixed_point",
    8:"Landing",
    9:"Looping",
}

class droneController:
    def __init__(self):
        self._debugCounter = 0
        self._status_debug = None

        self.zeroVelocity()

        rospy.init_node('drone_controller',anonymous = False)
        self._pubTwist = rospy.Publisher('/cmd_vel',Twist,queue_size = 50)
        self._rateTwist = rospy.Rate(50) #50Hz

        self._pubReset = rospy.Publisher('ardrone/reset',Empty,queue_size = 1)
        self._pubTakeoff = rospy.Publisher('ardrone/takeoff',Empty,queue_size = 1)
        self._pubLand = rospy.Publisher('ardrone/land',Empty,queue_size = 1)

        rospy.Subscriber('drone_controller/com', String, self.callbackCommand)
        rospy.Subscriber('ardrone/navdata', Navdata, self.callbackNavdata)

    def zeroVelocity(self):
        self._velocity = [0,0,0,0,0,0]

    def publishTwist(self):
        twist = Twist()
        twist.linear.x = self._velocity[0]
        twist.linear.y = self._velocity[1]
        twist.linear.z = self._velocity[2]
        twist.angular.x = self._velocity[3]
        twist.angular.y = self._velocity[4]
        twist.angular.z = self._velocity[5]

        self._pubTwist.publish(twist)
        

    def callbackNavdata(self,navdata):

        if self._status_debug is not droneStatus[navdata.state]:
            now = rospy.get_rostime()
            print("Drone status: " + droneStatus[navdata.state] + " Time: {} {}".format(now.secs, now.nsecs))

        self._status_debug = droneStatus[navdata.state]
        self._droneStatus = navdata.state

        #print("Is Flying?: " + str(self.isFlying()))
        self._batteryPercent = navdata.batteryPercent
        if self._batteryPercent < 20: print("%Battery: " + str(navdata.batteryPercent))

        ###DEBUG check sleep doesn't block navdata subscriber callback - seems it's OK, even if controller sleeps long time between publish twist msg callbackNavdata is called
        #self._debugCounter = self._debugCounter+1
        #print(self._debugCounter)
    
    def callbackCommand(self,data):
        msg = data.data
        now = rospy.get_rostime()
        #print("drone_controller: I heard " + msg + " Time: {} {}".format(now.secs, now.nsecs)+ "\n")
        tokenList = msg.split(' ')

        if tokenList[0] == "SET_VELOCITY":
            if controller.isFlying:
                self._velocity = [float(i) for i in tokenList[1:]]
                if self._velocity == [0,0,0,0,0,0]:
                    self._velocity[4] = 0.01 # prevent “auto hover”  mode
                    self._velocity[5] = 0.01 # prevent “auto hover”  mode
            

        elif tokenList[0] == "HOVER":
            self.zeroVelocity()

        elif tokenList[0] == "TAKEOFF":
            self._pubTakeoff.publish(Empty())
            self.zeroVelocity()

        elif tokenList[0] == "LAND":
            self._pubLand.publish(Empty())
            self.zeroVelocity()
            
        elif tokenList[0] == "RESET":
            self._pubReset.publish(Empty())
            self.zeroVelocity()

    def cleanup(self):
        print("droneController cleanup method")

    def sleep(self):
        self._rateTwist.sleep()

    def isFlying(self):
        return self._droneStatus == 3 or self._droneStatus == 7


if __name__ == "__main__":
    controller = droneController()

    try:
        while not rospy.is_shutdown():
            controller.publishTwist()
            controller.sleep()
    except rospy.ROSInterruptException:
        print("droneController: ROSInterruptException")
    finally:
        controller.cleanup()

