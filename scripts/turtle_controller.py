#!/usr/bin/env python

import rospy,roslib
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class turtleController:
    def __init__(self):
        self._velocity = (0,0,0,0,0,0) # linear velocity:xyz, angular velocity: xyz
        
    def setPublisher(self,publisher,rate):
        self._pub = publisher
        self._rate = rate

    def publishTwist(self):
        twist = Twist()
        twist.linear.x = self._velocity[0]
        twist.linear.y = self._velocity[1]
        twist.linear.z = self._velocity[2]
        twist.angular.x = self._velocity[3]
        twist.angular.y = self._velocity[4]
        twist.angular.z = self._velocity[5]

        self._pub.publish(twist)
        
    
    def callbackKeyboardCommand(self,data):
        msg = data.data
        print("I heard " + msg)
        tokenList = msg.split(' ')

        if tokenList[0] == "SET_VELOCITY":
            self._velocity = [float(i) for i in tokenList[1:]]

        elif tokenList[0] == "HOVER":
            self._velocity = (0,0,0,0,0,0)
            

    def cleanup(self):
        print("turtleController cleanup method")

    def sleep(self):
        self._rate.sleep()


if __name__ == "__main__":
    controller = turtleController()
    rospy.init_node('turtle_controller',anonymous = False)

    #set Publisher 
    controller.setPublisher(rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size = 1),rospy.Rate(50)) #50Hz

    #setSubscriber
    rospy.Subscriber('turtle_controller/com', String, controller.callbackKeyboardCommand)

    try:
        while not rospy.is_shutdown():
            controller.publishTwist()
            controller.sleep()
    except rospy.ROSInterruptException:
        print("turtleController: ROSInterruptException")
    finally:
        controller.cleanup()
