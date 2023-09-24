#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tkinter as tk
from tkinter import messagebox
from benthos_release.msg import BenthosReleaseCommand, BenthosReleaseStatus

# Some constants
BENTHOS_RELEASE_COMMAND_CHANNEL = rospy.get_param('~command_topic', "BENTHOS_RELEASE_COMMAND")
BENTHOS_RELEASE_STATUS_CHANNEL = rospy.get_param('~status_topic', "BENTHOS_RELEASE_STATUS")
AUTO_RANGE_TIME_DELAY = rospy.get_param('~auto_range_time_delay', 60)

class BenthosGuiNode:
    def __init__(self):
        rospy.init_node('benthos_gui_node', anonymous=True)

        self.cmd_msg = BenthosReleaseCommand()
        self.auto_ranging = False

        # Setup ROS Pubs & Subs
        self.command_pub = rospy.Publisher(BENTHOS_RELEASE_COMMAND_CHANNEL, BenthosReleaseCommand, queue_size=10)
        rospy.Subscriber(BENTHOS_RELEASE_STATUS_CHANNEL, BenthosReleaseStatus, self.status_handle)

        # Build GUI
        self.build_gui()

    def build_gui(self):
        self.gui = tk.Tk()
        self.frame = tk.Frame(self.gui)
        self.frame.grid()

        button = tk.Button(self.frame, text="Release 1", command=lambda: self.release(1))
        button.grid(row=0, column=0)
        button = tk.Button(self.frame, text="Release 2", command=lambda: self.release(2))
        button.grid(row=0, column=1)
        
        button = tk.Button(self.frame, text="Activate 1", command=lambda: self.activate(1))
        button.grid(row=1, column=0)
        button = tk.Button(self.frame, text="Activate 2", command=lambda: self.activate(2))
        button.grid(row=1, column=1)

        button = tk.Button(self.frame, text="Range 1", command=lambda: self.range_to(1))
        button.grid(row=2, column=0)
        button = tk.Button(self.frame, text="Range 2", command=lambda: self.range_to(2))
        button.grid(row=2, column=1)

        button = tk.Button(self.frame, text="Status 1", command=lambda: self.status(1))
        button.grid(row=3, column=0)
        button = tk.Button(self.frame, text="Status 2", command=lambda: self.status(2))
        button.grid(row=3, column=1)

        self.auto_range_btn = tk.Checkbutton(self.frame, text="Auto Range", command=self.auto_range)
        self.auto_range_btn.grid(row=4, column=0)
        
        self.time_entry = tk.Entry(self.frame, width=4)
        self.time_entry.grid(row=5, column=0)
        self.time_entry.insert(tk.END, str(AUTO_RANGE_TIME_DELAY))
        button = tk.Button(self.frame, text="Auto Range Delay (s)", command=self.time_auto_range)
        button.grid(row=5, column=1)

        label = tk.Label(self.frame, text="tx: ")
        label.grid(row=6, column=0)
        self.txlabel = tk.Label(self.frame, text="")
        self.txlabel.grid(row=6, column=1, columnspan=1)

        label = tk.Label(self.frame, text="rx: ")
        label.grid(row=7, column=0)
        self.rxlabel = tk.Label(self.frame, text="")
        self.rxlabel.grid(row=7, column=1, columnspan=1)
        
        self.gui.mainloop()
        
    def release(self, release_id):
        if messagebox.askyesno('Verify Release', 'Are you sure?'):
            if messagebox.askyesno('Be ABSOLUTELY sure', 'Really?'):
                self.cmd_msg.timestamp = rospy.Time.now()
                self.cmd_msg.id = release_id
                self.cmd_msg.command = "release"
                self.command_pub.publish(self.cmd_msg)
                rospy.loginfo('Releasing %d.' % release_id)
                self.txlabel.config(text='Releasing %d.' % release_id)
            else:
                self.txlabel.config(text='Coward.')
        else:
            self.txlabel.config(text='Literally, be sure next time!')

    def activate(self, release_id):
        self.cmd_msg.timestamp = rospy.Time.now()
        self.cmd_msg.id = release_id
        self.cmd_msg.command = "activate_release"
        self.command_pub.publish(self.cmd_msg)
        rospy.loginfo('Activating %d.' % release_id)
        self.txlabel.config(text='Activating %d.' % release_id)

    def status(self, release_id):
        self.cmd_msg.timestamp = rospy.Time.now()
        self.cmd_msg.id = release_id
        self.cmd_msg.command = "confirm_release"
        self.command_pub.publish(self.cmd_msg)
        rospy.loginfo('Confirming %d.' % release_id)
        self.txlabel.config(text='Confirming %d.' % release_id)

    def range_to(self, release_id): 
        self.cmd_msg.timestamp = rospy.Time.now()
        self.cmd_msg.id = release_id
        self.cmd_msg.command = "poll_range"
        self.command_pub.publish(self.cmd_msg)
        rospy.loginfo('Ranging %d.' % release_id)
        self.txlabel.config(text='Ranging %d.' % release_id)

    def auto_range(self): 
        if self.auto_ranging == False:
            self.auto_ranging = True
            rospy.loginfo('Automatically Ranging')
            self.txlabel.config(text='Auto Ranging')
        else:
            self.auto_ranging = False
            rospy.loginfo('Stopped Automatically Ranging')
            self.txlabel.config(text='Stopped Auto Ranging')

        self.cmd_msg.timestamp = rospy.Time.now()
        self.cmd_msg.id = 0
        self.cmd_msg.command = "auto_range"
        self.command_pub.publish(self.cmd_msg)

    def time_auto_range(self):
        timing = str(self.time_entry.get())
        if len(timing) == 1:
            self.cmd_msg.command = 'auto_time 000' + str(timing)
        elif len(timing) == 2:
            self.cmd_msg.command = 'auto_time 00' + str(timing)
        elif len(timing) == 3:
            self.cmd_msg.command = 'auto_time 0' + str(timing)
        elif len(timing) == 4:
            self.cmd_msg.command = 'auto_time ' + str(timing)
        self.cmd_msg.timestamp = rospy.Time.now()
        self.cmd_msg.id = 1
        rospy.loginfo(self.cmd_msg.command[-4:])
        self.command_pub.publish(self.cmd_msg)
        rospy.loginfo('Reset Timing to ' + timing)

    def status_handle(self, msg):
        rospy.loginfo("Received status message: %s" % msg)
        # Update GUI as needed with the received status message, below might work?
        self.rxlabel.config(text="Received: %s" % msg)

def main():
    try:
        BenthosGuiNode()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
