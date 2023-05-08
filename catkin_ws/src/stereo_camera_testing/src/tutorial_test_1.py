"""#!/usr/bin/python3.8
"""
import rospy
import depthai as dai
from depthai_sdk import OakCamera


def main():

    with OakCamera(usb_speed=dai.UsbSpeed.HIGH) as oak:
        color = oak.create_camera('color')
        left = oak.create_camera('left')
        right = oak.create_camera('right')
        oak.visualize([color, left, right], fps=True)
        oak.start(blocking=True)
 

if __name__ == "__main__":
    rospy.init_node("tutorial_test_1", anonymous=False)
    main()
