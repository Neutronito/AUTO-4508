#!/usr/bin/env python3

import rospy 
from kivymd.app import MDApp 
from kivy.lang import Builder 


class guiApp(MDApp):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.screen=Builder.load_file('~/catkin_ws/src/user_interface/scripts/ros_gui.kv')

    def build(self):
        return self.screen

if __name__ == '__main__':
    rospy.init_node('robot_gui', anonymous=True)

    guiApp().run()