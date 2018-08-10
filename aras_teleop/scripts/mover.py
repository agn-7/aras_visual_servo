#!/usr/bin/env python

import getch
import rospy

from std_msgs.msg import Float64


KEY_UP = 65
KEY_DOWN = 66
KEY_RIGHT = 67
KEY_LEFT = 68
USER_QUIT = 100

MAX_FORWARD = 2
MAX_LEFT = 2
MIN_FORWARD = -2
MIN_LEFT = -2


forward = 0.0
left = 0.0
keyPress = 0


if __name__ == '__main__':
    rospy.init_node('aras_teleop')

    while keyPress != USER_QUIT:
        pub1 = rospy.Publisher('/aras_visual_servo/joint2_position_controller/command', Float64, queue_size=1)
        pub2 = rospy.Publisher('/aras_visual_servo/gantry_position_controller/command', Float64, queue_size=1)

        _gantry = Float64()
        _joint1 = Float64()

        keyPress = getch.getArrow()

        if (keyPress == KEY_UP) and (forward <= MAX_FORWARD):
            forward += 0.05
        elif (keyPress == KEY_DOWN) and (forward >= MIN_FORWARD):
            forward -= 0.05
        elif (keyPress == KEY_LEFT) and (left <= MAX_LEFT):
            left += 0.05
        elif (keyPress == KEY_RIGHT) and (left >= MIN_LEFT):
            left -= 0.05

        _gantry.data = left
        _joint1.data = forward
        pub1.publish(_joint1)
        pub2.publish(_gantry)

    exit()
