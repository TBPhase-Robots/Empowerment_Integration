from tkinter import *
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from model.Publisher import Publisher
from std_msgs.msg import String
class Window(Frame):

    def __init__(self, master=None):
        Frame.__init__(self, master)        
        self.master = master

        # widget can take all window
        self.pack(fill=BOTH, expand=1)

        # create button, link it to clickExitButton()
        exitButton = Button(self, text="Exit", command=self.clickExitButton)

        # place button at (0,0)
        exitButton.place(x=0, y=0)

        # define command publisher
        commandListenerTopicName = "/controller/command"

        self.publisher = Publisher(commandListenerTopicName) 
        self.i = 0

    def clickExitButton(self):

        msg = String()
        self.i += 1
        msg.data = 'Hello World: {0}'.format(self.i)

        
        self.publisher.pub.publish(msg)

        print(msg)
        #exit()

        


rclpy.init(args=None)

root = Tk()
app = Window(root)
root.wm_title("Tkinter button")
root.geometry("320x200")
root.mainloop()

    
