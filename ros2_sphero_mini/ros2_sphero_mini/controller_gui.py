#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Vector3
import tkinter as tk
import math
import numpy as np

class TeleopGUI(Node):
    def __init__(self):
        super().__init__('sphero_teleop')

        # Publishers for GUI topics
        self.pub_dpad     = self.create_publisher(String,   '/GUI/dpad',      1)
        self.pub_joy      = self.create_publisher(Vector3,  '/GUI/joystick',  1)
        self.pub_slider   = self.create_publisher(Vector3,  '/GUI/slider',    1)
        self.pub_autonomy = self.create_publisher(Bool,     '/GUI/autonomy',  1)
        self.pub_stab    = self.create_publisher(Bool, '/GUI/stabilization', 1)
        self.pub_reset_h = self.create_publisher(Bool, '/GUI/reset_heading', 1)

        # Start ROS spin in background on a separate thread so callbacks work
        threading.Thread(target=rclpy.spin, args=(self,), daemon=True).start()

        # Build the Tkinter GUI
        self.build_gui()

    def build_gui(self):
        root = tk.Tk() # main window
        root.title("Sphero Teleop")

        # Joystick
        joy = self.Joystick(root, size=100, pub=self.pub_joy)
        joy.pack(side="left", padx=20, pady=20)

        # D-Pad
        dpad = self.DPad(root, size=100, pub=self.pub_dpad)
        dpad.pack(side="right", padx=20, pady=20)

        # Color Sliders
        sliders = self.ColorSelector(root, pub=self.pub_slider)
        sliders.pack(side="left", padx=20, pady=20)

        # Autonomy buttons
        buttons = self.Buttons(root,pub_autonomy=self.pub_autonomy,pub_stab=self.pub_stab,pub_reset_heading=self.pub_reset_h)
        buttons.pack(side="left", padx=20, pady=20)

        root.mainloop()  # run tkinter mainloop

    class Joystick(tk.Canvas):
        def __init__(self, master, size, pub):
            super().__init__(master, width=size, height=size, bg='lightgray')
            self.size, self.center = size, size/2
            self.pub = pub
            self.msg = Vector3()
            # self.create_oval ( x0,y0,x1,y1)
            self.handle = self.create_oval(size/4, size/4, 3*size/4, 3*size/4, fill='red')
            self.bind("<B1-Motion>", self.move) # fires continuously whenever mouse moves while button is held down
            self.bind("<ButtonRelease-1>", self.reset) # fires once when left mouse button is let go

        def move(self, event):
            dx, dy = event.x-self.center, event.y-self.center
            r = min(self.center, math.hypot(dx,dy)) # limits r, so never exceeds widget radius
            theta = math.atan2(dy,dx) # angle from pos x-axis to mouse cursor point
            # where to position the handle
            self.coords(self.handle,
                        self.center + r*math.cos(theta) - self.size/8,
                        self.center + r*math.sin(theta) - self.size/8,
                        self.center + r*math.cos(theta) + self.size/8,
                        self.center + r*math.sin(theta) + self.size/8)
            self.msg.x = float(r*3)
            self.msg.y = float(theta*180/np.pi + 90) # +90 offsets so that 0 deg is straight up, not right (what sphero expects)
            self.msg.z = 0.0
            self.pub.publish(self.msg)

        def reset(self, _):
            self.coords(self.handle,
                        self.center-self.size/8,
                        self.center-self.size/8,
                        self.center+self.size/8,
                        self.center+self.size/8)
            self.msg.x = self.msg.y = self.msg.z = 0.0
            self.pub.publish(self.msg)

    class DPad(tk.Canvas):
        def __init__(self, master, size, pub):
            super().__init__(master, width=size, height=size, bg='lightgray')
            self.size = size
            self.pub = pub
            self.msg = String()
            self.draw()
            self.bind("<Button-1>", self.click)

        def draw(self):
            s, h = self.size, self.size/2
            self.create_rectangle(0,0,s,s, fill='lightgray')
            # up arrow
            self.create_polygon(h-10,10, h,30, h+10,10, fill='red')
            # right arrow
            self.create_polygon(s-10,h-10, s-30,h, s-10,h+10, fill='red')
            # down arrow
            self.create_polygon(h-10,s-10, h,s-30, h+10,s-10, fill='red')
            # left arrow
            self.create_polygon(10,h-10, 30,h, 10,h+10, fill='red')

        def click(self, event):
            x,y = event.x, event.y
            s=self.size
            h=s/2
            # NOTE: x increases to the right, y down
            if y< h/2:        d="Up"
            elif y> 3*h/2:    d="Down"
            elif x< h/2:      d="Left"
            elif x> 3*h/2:    d="Right"
            else:             d="None"
            self.msg.data = d
            print(d)
            self.pub.publish(self.msg)

    class ColorSelector(tk.Frame):
        # Frame with three sliders for R, G, B values; publishes Vector3 on change.
        def __init__(self, master, pub):
            super().__init__(master)
            self.pub = pub; 
            for i,col in enumerate(('R','G','B')):
                s = tk.Scale(self, from_=0,to=255, orient='horizontal',
                             label=col, command=self.update)
                s.pack()
                setattr(self, col, s)

        def update(self, _):
            r = float(self.R.get())
            g = float(self.G.get())
            b = float(self.B.get())
            msg = Vector3()
            msg.x = r
            msg.y = g
            msg.z = b
            print(f"R: {r}, G: {g}, B: {b}")

            self.pub.publish(msg)

    class Buttons(tk.Frame):
        def __init__(self, master, pub_autonomy, pub_stab, pub_reset_heading):
            super().__init__(master)

            self.pub_autonomy       = pub_autonomy
            self.pub_stab           = pub_stab
            self.pub_reset_heading  = pub_reset_heading
            self.msg = Bool()
            tk.Button(self, text="Tracking", command=self.track).pack(side='left')
            tk.Button(self, text="Teleop", command=self.teleop).pack(side='left')
            tk.Button(self, text="Stabilization Off", command=self.off).pack(side='left')
            tk.Button(self, text="Stabilization On", command=self.on).pack(side='left')
            tk.Button(self, text="Reset Heading", command=self.reset).pack(side='left')

        def on(self):
            self.msg.data = True 
            self.pub_stab.publish(self.msg)
            # print("stab on")

        def off(self):
            self.msg.data = False 
            self.pub_stab.publish(self.msg)
            # print("stab off")

        def reset(self):
            self.msg.data = True 
            self.pub_reset_heading.publish(self.msg)

        def track(self):
            self.msg.data = True  
            self.pub_autonomy.publish(self.msg)
            print("track")
        def teleop(self):
            self.msg.data = False 
            self.pub_autonomy.publish(self.msg)
            print("teleop")

def main(args=None):
    rclpy.init(args=args)
    TeleopGUI()
    # when GUI closes, shutdown ROS
    rclpy.shutdown()

if __name__=='__main__':
    main()
