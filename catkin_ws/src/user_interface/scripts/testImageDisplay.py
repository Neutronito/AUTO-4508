import rospy
import tkinter as tk
from tkinter import *
from PIL import Image, ImageTk
import os
 
class PicsDisplay:
 
    def __init__(self):
        #Make window
        self.window = tk.Tk()
 
        #Centers Window To Screen
        self.screenwidth = self.window.winfo_screenwidth()
        self.screenheight = self.window.winfo_screenheight()
        self.place_width = str(int(self.screenwidth / 2 - 400))
        self.place_height = str(int(self.screenheight / 2 - 329))
        self.window.geometry("+" + self.place_width + "+" + self.place_height)
 
        #Initial Window Setup
        self.window.title("pioneer pics :D")
        self.window.resizable(width=False, height=False)
 
        #Variable Declaration
        # self.avariable = 1
        self.photoindex = 0
        self.currentphoto = ""
        self.photofactor = 0
        self.currentphotopath = ""
        self.currentphotoready = ""
        self.folderpath = ""
        self.lastfolderpath = ""
        self.photopath = ""
        self.photolist = []
        self.cwd = "C:/Users/bridge/Desktop/CompVision" #"~/AUTO-4508/catkin_ws/user_interface/scripts"
        self.display_x = 0
        self.display_y = 0
        self.currentrotation = 0
 
        #Widget Layout
        self.setup()
        
        #Initiates Main Loop
        self.window.mainloop()
 
    def setup(self):
        global distances_array, distance_count, image_count
        print("distance count")
        print(distance_count)
        print("image count")
        print(image_count)

        b_border = 10
 
        self.frame1 = tk.Frame(self.window)
        self.frame1.pack(side=tk.TOP)
 
        self.frame2 = tk.Frame(self.window)
        self.frame2.pack(side=tk.TOP)
        
        self.photoframe = tk.Frame(self.frame1, width=600, height=600)
        self.photoframe.pack(side=tk.LEFT)
        self.photoframe.pack_propagate(0)
 
        self.photodisplay = tk.Label(self.photoframe, text="no pics yet",
                                font=("Arial", 14))
        self.photodisplay.place(x=self.sizing_frame()[2],
                                y=self.sizing_frame()[3],
                                width=1000, height=1000)
 
        self.bottomframe = tk.Frame(self.frame2, width=600, height=100, bd=5,
                                    relief=tk.GROOVE)
        self.bottomframe.pack(side=tk.TOP)
        self.photoframe.pack_propagate(0)

        self.text=Text(self.bottomframe, width=80, height=2)
        self.text.pack(side=tk.TOP, padx=b_border)

        self.previousbutton = tk.Button(self.bottomframe, text="previous pic",
                                        font=("Arial", 12), width=15,
                                        command=self.previous)
        self.previousbutton.pack(side=tk.LEFT, anchor=tk.W,
                                 padx=b_border)
        
        self.distancebutton = tk.Button(self.bottomframe, text="show distance",
                                        font=("Arial", 12), width=15,
                                        command=self.show_distance)
        self.distancebutton.pack(side=tk.LEFT, anchor=tk.W,
                                 padx=b_border)

        self.folderbutton = tk.Button(self.bottomframe, text="show pics",
                                      font=("Arial", 12), width=15, command=self.folderselect)
        self.folderbutton.pack(side=tk.LEFT, padx=b_border)
 
        self.nextbutton = tk.Button(self.bottomframe, text="next pic", font=(
            "Arial", 12), width=15, command=self.next)
        self.nextbutton.pack(side=tk.LEFT, padx=b_border)

    
    def show_distance(self):
        # next and previous functions would indicate which image we are on
        # show_distances will erase previous distance and replace it with new distance 

        global image_count, distance_count

        print("DISTANCES SIZE")
        print(image_count)
        self.text.delete('1.0', END)
        self.text.insert(END, distances_array[image_count+1])

    def sizing_photo(self):
        resolution = [600, 600]
        return resolution    
 
    def sizing_frame(self):
        resolution = [1000, 1000, -200, -200]
        return resolution
 
    def next(self):
        global image_count, distance_count 
        image_count = image_count + 1

        self.display_x = 0
        self.display_y = 0
        self.currentrotation = 0
        if self.currentphoto != None:
            self.currentphoto.close()
            self.currentphoto = None
        if self.photoindex < len(self.photolist) - 1:
            self.photoindex = self.photoindex + 1
        else:
            self.photoindex = 0
 
        self.reloadphoto()


    def previous(self):
        global image_count, distance_count
        image_count = image_count - 1

        self.display_x = 0
        self.display_y = 0
        self.currentrotation = 0
        if self.currentphoto != None:
            self.currentphoto.close()
            self.currentphoto = None
        if self.photoindex == 0:
            self.photoindex = len(self.photolist) - 1
        else:
            self.photoindex = self.photoindex - 1
 
        self.reloadphoto()
 
 
    def folderselect(self):
        self.folderpath = "/home/group2/Pictures/" # "../group2/catkin_dependencies/photos"
        if self.folderpath != "":
            self.photolist = []
            self.photoindex = 0
            self.display_x = 0
            self.display_y = 0
            self.currentphotoready = None
            self.cwd = self.folderpath
            for name in os.listdir(self.folderpath):
                if name.lower().endswith(".jpg"): # or name.lower().endswith(".png")
                    self.photolist.append(name)

            if self.photolist != []:
                self.reloadphoto()
            else:
                self.photodisplay["image"]=""
                self.photodisplay["text"]="no pics in folder :("
                print(self.photodisplay["text"], self.photodisplay["image"])

    def loadphoto(self):
        print(self.photoindex)
        self.photodisplay["image"]=""
        self.currentphotoready = ""
        name = self.photolist[self.photoindex]
        if self.folderpath != "":
            self.currentphotopath = self.folderpath + "/" + str(self.photolist[self.photoindex])
            self.currentphoto = Image.open(self.currentphotopath)
 
    def reloadphoto(self):
        self.loadphoto()
        self.photoscale()
        self.displayphoto()
     
    def displayphoto(self):
        self.photodisplay.place(width=self.sizing_frame()[0], height=self.sizing_frame()[1], x=self.sizing_frame()[2] + self.display_x, y=self.sizing_frame()[3] + self.display_y)
        self.currentphotoready = ImageTk.PhotoImage(self.currentphoto)
        self.photodisplay.config(image=self.currentphotoready)
         
    def photoscale(self):
        print(self.currentphoto.width, self.currentphoto.height)
        if self.currentphoto.width > self.currentphoto.height:
            self.photofactor = self.currentphoto.width / self.sizing_photo()[0]
        else:
            self.photofactor = self.currentphoto.height / self.sizing_photo()[1]
 
        self.currentphoto.thumbnail((int(self.currentphoto.width / self.photofactor), int(self.currentphoto.height / self.photofactor)), Image.LANCZOS) # or Image.ANTIALIAS
 
        print(self.currentphoto.width, self.currentphoto.height)
 
 
def main():
    global distance_count

    # read file line by line and store in array
    file1 = open('/home/group2/catkin_dependencies/distances.txt', 'r')
    Lines = file1.readlines()

    for line in Lines:
        distance_count += 1
        distances_array.append(line.strip())
        # print(distances_array[count-1])

    program = PicsDisplay()
 
if __name__ == "__main__":
    distances_array = []
    distance_count = 0 # global variable to track pics and distances 
    image_count = 0
    main()
