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
	       }
ToggleCommands = {
                '1':"Enable/Disable Follow Controller",
                '2':"Enable/Disable TakeOffUnit"
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
    pub = rospy.Publisher('/drone_controller/com', String, queue_size = 3)
    pubEmergency = rospy.Publisher('/ardrone/reset', Empty, queue_size = 3)
    pubLand = rospy.Publisher('/ardrone/land', Empty, queue_size = 3)
    pubTakeoff = rospy.Publisher('/ardrone/takeoff', Empty, queue_size = 3)
    pubEnableControl = rospy.Publisher('follow_controller/enable_control', Bool, queue_size = 3)
    pubEnableTakeOff = rospy.Publisher('take_off_unit/enable_control', Bool, queue_size=3)
    enableControlFlag = True
    enableTakeOff = True

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
        elif key in ToggleCommands:
            if ToggleCommands[key] == "Enable/Disable Follow Controller":
                print("received command from keyboard: Enable/Disable Follow Controller = {}".format(enableControlFlag) + "\n")
                pubEnableControl.publish(enableControlFlag)
                enableControlFlag = not enableControlFlag
            if ToggleCommands[key] == "Enable/Disable TakeOffUnit":
                print("received command from keyboard: Enable/Disable Take Off Unit = {}".format(enableTakeOff) + "\n")
                pubEnableTakeOff.publish(enableTakeOff)
                enableTakeOff = not enableTakeOff
		
        elif key == quitChar:
                break




