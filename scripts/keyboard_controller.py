#!/usr/bin/env python
import sys,select,termios,tty

import rospy,roslib
from std_msgs.msg import String, Empty, Bool


quitChar = 'q'
keyCommands = {
		'i':"SET_VELOCITY 0.1 0 0 0 0 0",
		'k':"SET_VELOCITY -0.1 0 0 0 0 0",
		'l':"SET_VELOCITY 0 0 0 0 0 -0.1",
		'j':"SET_VELOCITY 0 0 0 0 0 0.1",
                'h':"SET_VELOCITY 0 0 0.3 0 0 0",
                'b':"SET_VELOCITY 0 0 -0.3 0 0 0",
                'y':"SET_VELOCITY 0 -0.1 0 0 0 0",
                't':"SET_VELOCITY 0 0.1 0 0 0 0",
                'z':"TAKEOFF",
                'x':"LAND",
                'c':"HOVER",
                'v':"RESET",
        '2':"SET_VELOCITY 0 0 0 0.1 0.1 0"
	       }
followControllerCommands = {
                '1':"Enable/Disable Control"
                }

directCommands = {
                'e':"DIRECT_EMERGENCY",
                'w':"DIRECT_LAND",
                'r':"DIRECT_TAKEOFF",
                }

class _Getch:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    

if __name__ == "__main__":
    pub = rospy.Publisher('/drone_controller/com', String, queue_size = 1)
    pubEmergency = rospy.Publisher('/ardrone/reset', Empty, queue_size = 1)
    pubLand = rospy.Publisher('/ardrone/land', Empty, queue_size = 1)
    pubTakeoff = rospy.Publisher('/ardrone/takeoff', Empty, queue_size = 1)
    pubEnableControl = rospy.Publisher('follow_controller/enable_control', Bool, queue_size = 1)
    enableControlFlag = True

    rospy.init_node('keyboard_controller')
    getch = _Getch()

    while(1):
        key = getch()
        if key in keyCommands:
                command = keyCommands[key]
                
                print("received command from keyboard:" + "key: {0}, command: {1}".format(key,command) + "\n")

		pub.publish(command)
        elif key in directCommands:
            if directCommands[key] == "DIRECT_EMERGENCY":
                print("received command from keyboard: DIRECT_EMERGENCY \n")
                pubEmergency.publish(Empty())
            elif directCommands[key] == "DIRECT_LAND":
                print("received command from keyboard: DIRECT_LAND \n")
                pubLand.publish(Empty())
            elif directCommands[key] == "DIRECT_TAKEOFF":
                print("received command from keyboard: DIRECT_TAKEOFF \n")
                pubTakeoff.publish(Empty())
	elif key in followControllerCommands:
	    if followControllerCommands[key] == "Enable/Disable Control":
		print("received command from keyboard: Enable/Disable Control = {}".format(enableControlFlag) + "\n")
		pubEnableControl.publish(enableControlFlag)
		enableControlFlag = not enableControlFlag
		
        elif key == quitChar:
                break




