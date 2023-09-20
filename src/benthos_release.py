#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import select
import rospy
import time

# Import ROS messages
from urioce_lcm.msg import BenthosReleaseCommand, BenthosReleaseStatus

# Some constants
BENTHOS_RELEASE_COMMAND_CHANNEL = "BENTHOS_RELEASE_COMMAND"
BENTHOS_RELEASE_STATUS_CHANNEL = "BENTHOS_RELEASE_STATUS"
BENTHOS_RELEASE_ID_1 = 34
BENTHOS_RELEASE_ID_2 = 35
BENTHOS_RELEASE_CODE_1 = 39466
BENTHOS_RELEASE_CODE_2 = 39467
AUTO_RANGE_TIME_DELAY = 60
SOUND_VELOCITY = 1537.8

class BenthosReleaseNode:
    def __init__(self):
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

        # Start main loop
        self.main()

    def benthos_release_command_handler(self, msg):
        global AUTO_RANGE_TIME_DELAY

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
            AUTO_RANGE_TIME_DELAY = int(msg.command[-4:])
        else:
            rospy.logwarn("Unknown command: %s" % msg.command)
            return

    def release(self, benthos_id, release_code):
        # Implement your release logic here using ROS messages or other methods
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
        # Implement your confirm release logic here using ROS messages or other methods
        rospy.loginfo("Confirm release: ID=%d" % benthos_id)

    def poll_range(self, benthos_id):
        # Implement your poll range logic here using ROS messages or other methods
        rospy.loginfo("Polling range: ID=%d" % benthos_id)

    def main(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            recv, _, _ = select.select([self.command_sub], [], [], 1.0)

            if recv:
                # Got a command message, handle it
                msg = rospy.wait_for_message(BENTHOS_RELEASE_COMMAND_CHANNEL, BenthosReleaseCommand)
                self.benthos_release_command_handler(msg)

            else:
                # Handle timeout or auto-ranging
                if self.ranging == True:
                    if self.auto_range_count > int(AUTO_RANGE_TIME_DELAY):
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

            rate.sleep()

    def publish_status(self, id, range_value):
        status_msg = BenthosReleaseStatus()
        status_msg.timestamp = int(time.time() * 1e6)
        status_msg.id = id
        status_msg.range = range_value * SOUND_VELOCITY / 1500.
        self.status_pub.publish(status_msg)

if __name__ == "__main__":
    try:
        BenthosReleaseNode()
    except rospy.ROSInterruptException:
        pass
