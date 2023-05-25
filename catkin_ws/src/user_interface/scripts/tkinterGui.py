#! /usr/bin/env python
import rospy
from itertools import count 
from std_msgs.msg import String
from std_msgs.msg import Int16
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan

import tkinter 
from tkinter import *
import numpy as np
from matplotlib.figure import Figure
from matplotlib import pyplot as plt
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, 
NavigationToolbar2Tk)
from matplotlib.backend_bases import key_press_handler
import math 
from math import * 
import sympy as sym 
from sympy import * 
from matplotlib.animation import FuncAnimation

def find_gradient_c(x1, x2, y1, y2):
    global gradient 
    global c 

    gradient = (y2 - y1) / (x2 - x1)
    c = y1 - (x1*gradient)

def f(x, m, c):
    y = m*x + c
    return y

def f(x, m, c):
    m = -1/m
    y = m*x + c
    return y

def find_gps_distance(x1, x2, y1, y2):
    global gps_distance 

    gps_distance = math.sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))

def extract_real(string):
    global real_index 
    string2 = ""
    for i in range(len(string)-1, 0-1, -1):
        if(string[i] == "."):
            real_index = i - 4
            break
    
    for j in range(real_index):
        string2 = string2 + string[j]
        
    return string2

def gps_callback(msg):
	global curLatitude, curLongitude
	curLatitude = msg.latitude 
	curLongitude = msg.longitude 

def lidar_callback(msg):
	global lidar
	lidar = msg.data
    # data = 1 obstacle on right, -1 obstacle on left, 2 obstacle in front, 0 no obstacle wooohoooo

def scaleLongitude(longitude_array):
	global curLongitude 

	centre_longitude = 115.817685
	x = curLongitude - centre_longitude
	longitude_array.append(x)

def scaleLatitude(latitude_array):
	global curLatitude

	centre_latitude = -31.980205
	y = curLatitude - centre_latitude
	latitude_array.append(y)

def animate(i):
	global gps_distance, lidar_distance, curLongitude, curLatitude, lidar, count
        
	count = count + 1
        
	rospy.Subscriber("fix", NavSatFix, gps_callback)
	rospy.Subscriber("lidar_response", Int16, lidar_callback)
	rospy.Publisher('tkinter', String, queue_size=10) 

	scaleLongitude(longitude_array)
	scaleLatitude(latitude_array)

	# length= len(longitude_array)
	# x2 = longitude_array[length-1]
	# y2 = latitude_array[length-1]
	# x1 = longitude_array[length-2]
	# y1 = latitude_array[length-2]

	# find_gradient_c(x1, x2, y1, y2)
	# find_gps_distance(x1, x2, y1, y2)

	# if (lidar == 1): # if obstacle is on right 
	# 	gps_distance = gps_distance*gps_distance
	# 	x, y= sym.symbols('x,y')
	# 	eq1 = sym.Eq(sym.sqrt((x-x2)*(x-x2) + (y-y2)*(y-y2)), lidar_distance)
	# 	eq2 = sym.Eq((x-x1)*(x-x1) + (y-y1)*(y-y1), (x-x2)*(x-x2) + (y-y2)*(y-y2) + gps_distance)
	# 	result = sym.solve([eq1,eq2],(x,y))
	# 	print(len(result))

	# 	result_x1 = result[0][0]
	# 	result_y1 = result[0][1]
	# 	result_x2 = result[1][0]
	# 	result_y2 = result[1][1]

	# 	string_result_x1 = str(result_x1)
	# 	string_result_y1 = str(result_y1)
	# 	string_result_x2 = str(result_x2)
	# 	string_result_y2 = str(result_y2)

	# 	if(len(string_result_x1)) < 25:
	# 		if (y1 < y2):
	# 			if(result_x1 > result_x2):
	# 				x = result_x1
	# 				y = result_y1
	# 			else:
	# 				x = result_x2 
	# 				y = result_y2

	# 		if(y2 < y1):
	# 			if(result_x1 < result_x2):
	# 				x = result_x1
	# 				y = result_y1
	# 			else:
	# 				x = result_x2
	# 				y = result_y2

	# 	elif(len(string_result_x1)) >= 25:
	# 		result_x1 = str(result_x1)
	# 		result_x2 = str(result_x2)
	# 		result_y1 = str(result_y1)
	# 		result_y2 = str(result_y2)

	# 		result_x1 = float(result_x1)
	# 		result_x2 = float(result_x2)
	# 		result_y1 = float(result_y1)
	# 		result_y2 = float(result_y2)
            
	# 		if (y1 < y2):
	# 			if(result_x1 > result_x2):
	# 				x = result_x1
	# 				y = result_y1
	# 			else:
	# 				x = result_x2 
	# 				y = result_y2

	# 		if(y2 < y1):
	# 			if(result_x1 < result_x2):
	# 				x = result_x1
	# 				y = result_y1
	# 			else:
	# 				x = result_x2
	# 				y = result_y2

	# 	plt.scatter(x, y, c = "red")

	# if (lidar == -1): # if obstacle is on left
	# 	gps_distance = gps_distance*gps_distance
	# 	x, y= sym.symbols('x,y')
	# 	eq1 = sym.Eq(sym.sqrt((x-x2)*(x-x2) + (y-y2)*(y-y2)), lidar_distance)
	# 	eq2 = sym.Eq((x-x1)*(x-x1) + (y-y1)*(y-y1), (x-x2)*(x-x2) + (y-y2)*(y-y2) + gps_distance)
	# 	result = sym.solve([eq1,eq2],(x,y))

	# 	rospy.loginfo(len(result))

	# 	result_x1 = result[0][0]
	# 	result_y1 = result[0][1]
	# 	result_x2 = result[1][0]
	# 	result_y2 = result[1][1]

	# 	string_result_x1 = str(result_x1)
	# 	string_result_y1 = str(result_y1)
	# 	string_result_x2 = str(result_x2)
	# 	string_result_y2 = str(result_y2)

	# 	if(len(string_result_x1)) < 25:
	# 		if (y1 < y2):
	# 			if(result_x1 < result_x2):
	# 				x = result_x1
	# 				y = result_y1
	# 			else:
	# 				x = result_x2 
	# 				y = result_y2

	# 		if(y2 < y1):
	# 			if(result_x1 > result_x2):
	# 				x = result_x1
	# 				y = result_y1
	# 			else:
	# 				x = result_x2
	# 				y = result_y2

	# 	elif(len(string_result_x1)) >= 25:
	# 		result_x1 = str(result_x1)
	# 		result_x2 = str(result_x2)
	# 		result_y1 = str(result_y1)
	# 		result_y2 = str(result_y2)

	# 		result_x1 = float(result_x1)
	# 		result_x2 = float(result_x2)
	# 		result_y1 = float(result_y1)
	# 		result_y2 = float(result_y2)
            
	# 		if (y1 < y2):
	# 			if(result_x1 < result_x2):
	# 				x = result_x1
	# 				y = result_y1
	# 			else:
	# 				x = result_x2 
	# 				y = result_y2

	# 		if(y2 < y1):
	# 			if(result_x1 > result_x2):
	# 				x = result_x1
	# 				y = result_y1
	# 			else:
	# 				x = result_x2
	# 				y = result_y2

	# 	plt.scatter(x, y, c = "red")

	# if (lidar == 2):
	# 	x, y= sym.symbols('x,y')
	# 	eq1 = sym.Eq(sym.sqrt((x-x1)*(x-x1) + (y-y1)*(y-y1)), gps_distance + lidar_distance)
	# 	eq2 = sym.Eq(sym.sqrt((x-x2)*(x-x2) + (y-y2)*(y-y2)),lidar_distance)
	# 	result = sym.solve([eq1,eq2],(x,y))
	# 	print(len(result))

	# 	result_x1 = result[0][0]
	# 	result_y1 = result[0][1]
	# 	result_x2 = result[1][0]
	# 	result_y2 = result[1][1]

	# 	string_result_x1 = str(result_x1)
	# 	string_result_y1 = str(result_y1)
	# 	string_result_x2 = str(result_x2)
	# 	string_result_y2 = str(result_y2)

	# 	if(len(string_result_x1)) < 25:
	# 		y = f(result_x1, gradient, c)
	# 		if(result_y1 == y):
	# 			x = result_x1
	# 			y = result_y1
	# 		else:
	# 			x = result_x2
	# 			y = result_y2
        
	# 	elif(len(string_result_x1)) >= 25:    
	# 		result_x1 = str(result_x1)
	# 		result_x2 = str(result_x2)
	# 		result_y1 = str(result_y1)
	# 		result_y2 = str(result_y2)

	# 		result_x1 = extract_real(result_x1)
	# 		result_x2 = extract_real(result_x2)
	# 		result_y1 = extract_real(result_y1)
	# 		result_y2 = extract_real(result_y2)

	# 		result_x1 = float(result_x1)
	# 		result_x2 = float(result_x2)
	# 		result_y1 = float(result_y1)
	# 		result_y2 = float(result_y2)

	# 		y = f(result_x1, gradient, c)
	# 		if(result_y1 == y):
	# 			x = result_x1
	# 			y = result_y1
	# 		else:
	# 			x = result_x2
	# 			y = result_y2

	# 	plt.scatter(x, y, c = "red")

	
	if(count == 2):
		longitude_array.pop(0)
		longitude_array.pop(0)
		latitude_array.pop(0)
		latitude_array.pop(0)
    
	if(count > 2):
		plt.plot(longitude_array, latitude_array)
                

rospy.init_node('tkinterGUI')
plt.style.use('grayscale')

longitude_array = [] 
latitude_array = [] 
gps_distance = 0.0
lidar_distance = 0.00005
curLatitude = 0.0
curLongitude = 0.0
real_index = 0
lidar = 0
count = 0

xmin, xmax, ymin, ymax = -0.000545, 0.000545, -0.000835, 0.000835
fig, ax = plt.subplots(figsize=(8, 8))
ax.set(xlim=(xmin, xmax), ylim=(ymin, ymax), aspect='equal')
ax.spines['bottom'].set_position('zero')
ax.spines['left'].set_position('zero')
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)

root = tkinter.Tk()
root.wm_title("Robot Path")

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.draw()

toolbar = NavigationToolbar2Tk(canvas, root)
toolbar.update()

canvas.mpl_connect(
    "key_press_event", lambda event: print(f"you pressed {event.key}"))
canvas.mpl_connect("key_press_event", key_press_handler)

button = tkinter.Button(master=root, text="Quit", command=root.quit)
button.pack(side=tkinter.BOTTOM)

toolbar.pack(side=tkinter.BOTTOM, fill=tkinter.X)
canvas.get_tk_widget().pack(side=tkinter.TOP, fill=tkinter.BOTH, expand=1)

ani = FuncAnimation(plt.gcf(), animate, interval=1000) # 3 secs?

tkinter.mainloop()








