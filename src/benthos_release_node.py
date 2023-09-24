#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time

# Import ROS messages
from benthos_release.msg import BenthosReleaseCommand, BenthosReleaseStatus
from udp_tools_ros import serialUDP

# Get ROS param constants
BENTHOS_RELEASE_COMMAND_CHANNEL = rospy.get_param('~command_topic', "BENTHOS_RELEASE_COMMAND")
BENTHOS_RELEASE_STATUS_CHANNEL = rospy.get_param('~status_topic', "BENTHOS_RELEASE_STATUS")
BENTHOS_RELEASE_ID_1 = rospy.get_param('~release_id1', 34)
BENTHOS_RELEASE_ID_2 = rospy.get_param('~release_id2', 35)
BENTHOS_RELEASE_CODE_1 = rospy.get_param('~release_code1', 39466)
BENTHOS_RELEASE_CODE_2 = rospy.get_param('~release_code2', 39467)
SOUND_VELOCITY = rospy.get_param('~sound_velocity', 1537.8)
SAMPLE_RATE = rospy.get_param('~sample_rate', 1.0) # 1 Hz default

class BenthosReleaseNode:
    def __init__(self):
        # Parse command line arguments and configure serial port.
        # This class can communicate over hardware serial or through a network converter
        # -b baudrate -D device
        # Initialize ROS node
        rospy.init_node("benthos_release_node")

        # Create ROS publishers and subscribers
        self.command_sub = rospy.Subscriber(BENTHOS_RELEASE_COMMAND_CHANNEL, BenthosReleaseCommand, self.benthos_release_command_handler)
        self.status_pub = rospy.Publisher(BENTHOS_RELEASE_STATUS_CHANNEL, BenthosReleaseStatus, queue_size=10)

        # Initialize other variables
        self.ranging = False
        self.id_ranging_to = BENTHOS_RELEASE_ID_1
        self.auto_range_count = 0
        self.time_since_last = 0
        self.auto_range_time_delay = rospy.get_param('~auto_range_time_delay', 60)


        # Initialize message instances
        self.status_msg = BenthosReleaseStatus()
        self.command_msg = BenthosReleaseCommand()

        # Initialize serial to UDP class instance
        self.comm = serialUDP.serialUDP()

        # Start main loop
        self.main()

    def benthos_release_command_handler(self, msg):

        if msg.id == 1:
            benthos_id = BENTHOS_RELEASE_ID_1
            release_code = BENTHOS_RELEASE_CODE_1
        elif msg.id == 2:
            benthos_id = BENTHOS_RELEASE_ID_2
            release_code = BENTHOS_RELEASE_CODE_2
        elif msg.id == 0:
            pass
        else:
            rospy.logwarn("Invalid ID %d" % msg.id)
            return

        if msg.command == "release":
            self.release(benthos_id, release_code)
        elif msg.command == "poll_range":
            self.poll_range(benthos_id)
        elif msg.command == "confirm_release":
            self.confirm_release(benthos_id)
        elif msg.command == "activate_release":
            self.activate_release(benthos_id)
        elif msg.command == "auto_range":
            self.auto_range()
        elif msg.command.startswith("auto_time"):
            rospy.loginfo('Auto Range Timing change to %d' % int(msg.command[-4:]))
            self.auto_range_time_delay = int(msg.command[-4:])
        else:
            rospy.logwarn("Unknown command: %s" % msg.command)
            return

    def release(self, benthos_id, release_code):
        serial_cmd = "AT%R^" + str(benthos_id) + "," + str(release_code) + "\n"
        self.send_to_benthos(serial_cmd)
        rospy.loginfo("Release command: ID=%d, Code=%d" % (benthos_id, release_code))

    def activate_release(self, benthos_id):
        # Implement your activate release logic here using ROS messages or other methods
        rospy.loginfo("Activate release: ID=%d" % benthos_id)

    def auto_range(self):
        if self.ranging == False:
            self.id_ranging_to = BENTHOS_RELEASE_ID_1
            self.ranging = True
            self.poll_range(self.id_ranging_to)
        elif self.ranging == True:
            rospy.loginfo('Stopping Auto Ranging')
            self.id_ranging_to = 0
            self.ranging = False
            self.auto_range_count = 0

    def confirm_release(self, benthos_id):
        rospy.loginfo("Confirm release: ID=%d" % benthos_id)
        serial_cmd = "AT%RT" + str(benthos_id) + "\n"
        self.send_to_benthos(serial_cmd)

    def poll_range(self, benthos_id):
        rospy.loginfo("Polling range: ID=%d" % benthos_id)
        serial_cmd = "AT%RR" + str(benthos_id) + "\n"
        self.send_to_benthos(serial_cmd)

    def send_to_benthos(self, serial_cmd):
        self.comm.send(serial_cmd)
        rospy.loginfo("Sent command: %s" % serial_cmd)

    def read_range(self, line):
        a = line.split(':')
        benthos_range = float(a[1][1:-2])
        benthos_id = int(a[0][-2:])

        if benthos_id == BENTHOS_RELEASE_ID_1:
            self.id = 1
        if benthos_id == BENTHOS_RELEASE_ID_2:
            self.id = 2

        # stuff remaining data
        self.range = benthos_range * SOUND_VELOCITY / 1500.

        # publish data
        self.publish_status(self)
        rospy.loginfo("Published to %s. Range: %f at %d from %d" % (BENTHOS_RELEASE_STATUS_CHANNEL, self.status_msg.range, self.status_msg.timestamp, self.status_msg.id))

    def publish_status(self):
        self.status_msg.timestamp = int(time.time() * 1e6)
        self.status_msg.id = self.id
        self.status_msg.range = self.range * SOUND_VELOCITY / 1500.
        self.status_pub.publish(self.status_msg)

    def serial_handler(self):
        line = self.comm.readline()     # read the serial input line and set it equal to line

        if line.strip() == "RELEASED and CONFIRMED":
            self.read_released(line)

        elif line.strip() == "Response Not Received":  # this is only for status updates not received
            rospy.loginfo("Response not received")

        elif line.strip().__len__() > 5 and line.strip()[0:5] == "Range":
            self.time_since_last = 0
            rospy.loginfo("Probably a range")
            self.read_range(line)

        elif line.strip() == "Invalid Command":
            rospy.loginfo("Invalid command")

        else:
            rospy.loginfo("Unhandled serial string")

    def main(self):
        rate = rospy.Rate(SAMPLE_RATE)
        while not rospy.is_shutdown():

            self.serial_handler()

            if self.ranging == True:
                if self.auto_range_count > int(self.auto_range_time_delay):
                    if self.id_ranging_to == BENTHOS_RELEASE_ID_1:
                        self.id_ranging_to = BENTHOS_RELEASE_ID_2
                    else:
                        self.id_ranging_to = BENTHOS_RELEASE_ID_1
                    self.poll_range(self.id_ranging_to)
                    self.auto_range_count = 0
                else:
                    self.auto_range_count += 1

            self.time_since_last += 1
            if self.time_since_last >= 600 and self.time_since_last < 605:
                self.publish_status(0, 1)
            elif self.time_since_last >= 1200 and self.time_since_last < 1205:
                self.publish_status(0, 2)

            # if id is set to 0 that means change the background color of gui
            # the range will be the background color
            # 0 is nothing is wrong
            # 1 is start to worry
            # 2 is stress out now

            rate.sleep()

if __name__ == "__main__":
    try:
        BenthosReleaseNode()
    except rospy.ROSInterruptException:
        pass
