"""#!/usr/bin/python3.8
"""
import rospy
from depthai_sdk import OakCamera

def main():

    with OakCamera() as oak:
        color = oak.create_camera('color')
        left = oak.create_camera('left')
        right = oak.create_camera('right')
        oak.visualize([color, left, right], fps=True)
        oak.start(blocking=True)
 

if __name__ == "__main__":
    rospy.init_node("tutorial_test", anonymous=False)
    main()
