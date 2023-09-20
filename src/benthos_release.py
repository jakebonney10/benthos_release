#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#!/usr/bin/python

# import the necessary libraries and codes
import select, lcm, time # base python stuff

# our serial library and relevant LCM types
import serialUDP
from urioce_lcm import benthos_release_command_t, benthos_release_status_t

# some constants
BENTHOS_RELEASE_COMMAND_CHANNEL = "BENTHOS_RELEASE_COMMAND" # the name of the LCM channels
BENTHOS_RELEASE_STATUS_CHANNEL = "BENTHOS_RELEASE_STATUS"
BENTHOS_RELEASE_ID_1 = 34   # full ID # 72234
BENTHOS_RELEASE_ID_2 = 35   # full ID # 72235
BENTHOS_RELEASE_CODE_1 = 39466
BENTHOS_RELEASE_CODE_2 = 39467
AUTO_RANGE_TIME_DELAY = 60

SOUND_VELOCITY = 1537.8


class benthos_release:
    def __init__(self, lc):
        # Parse command line arguments and configure serial port.
        # This class can communicate over hardware serial or through a network converter
        # -b baudrate -D device
        self.comm = serialUDP.serialUDP()
        self.lc = lc
        self.lc.subscribe(BENTHOS_RELEASE_COMMAND_CHANNEL, self.benthos_release_command_handler)
        self.lcm_status = benthos_release_status_t()
        self.ranging = False
        self.id_ranging_to = BENTHOS_RELEASE_ID_1
        self.auto_range_count = 0
        self.time_since_last = 0

    def benthos_release_command_handler(self, channel, data):
        global AUTO_RANGE_TIME_DELAY
        # Attempt to decode LCM message
        try:
            msg = benthos_release_command_t.decode(data)
        except ValueError:
            print("Failed to decode LCM message.")
            return

        # Translate from a human-friendly ID (1 vs 2, port vs stbd, etc) to the actual addresses
        if msg.id == 1:
            benthos_id = BENTHOS_RELEASE_ID_1
            release_code = BENTHOS_RELEASE_CODE_1
        elif msg.id == 2:
            benthos_id = BENTHOS_RELEASE_ID_2
            release_code = BENTHOS_RELEASE_CODE_2
        elif msg.id == 0:
            pass
        else:
            print("Invalid ID %d" % msg.id)
            return

        # Identify possible commands and call the appropriate function
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
        elif msg.command[0:9] == "auto_time":
            print 'Auto Range Timing change to %d' %(int(msg.command[-4:]))
            AUTO_RANGE_TIME_DELAY = int(msg.command[-4:])
        else:
            print("Unknown command: %s" % msg.command)
            return

    def release(self, benthos_id, release_code):
        serial_cmd = "AT%R^" + str(benthos_id) + "," + str(release_code) + "\n"
        self.send_to_benthos(serial_cmd)
        
    def activate_release(self, benthos_id):
        print('Please Acitvate through TopSide interface')
#        serial_cmd = "AT%RA" + str(benthos_id) + "\n"
#        self.send_to_benthos(serial_cmd)
        
        
# AT%RA<ID>,nn,w Activate
        

    def auto_range(self):
        if self.ranging == False:
            self.id_ranging_to = BENTHOS_RELEASE_ID_1
            self.ranging = True
            self.poll_range(self.id_ranging_to)
        elif self.ranging == True:
            print('Stopping Auto Ranging')
            self.id_ranging_to = 0
            self.ranging = False
            self.auto_range_count = 0
        
    def confirm_release(self, benthos_id):
        serial_cmd = "AT%RT" + str(benthos_id) + "\n"
        self.send_to_benthos(serial_cmd)

    def poll_range(self, benthos_id):
        # Don't wait for a response. It wastes CPU time and you're never sure if the next serial message will be what you expect. 
        # If and when you get an answer, the serial handler will get it.
        serial_cmd = "AT%RR" + str(benthos_id) + "\n"
        self.send_to_benthos(serial_cmd)
        
    def send_to_benthos(self, serial_cmd):
        self.comm.send(serial_cmd)
        print("Sent command: %s" % serial_cmd)

    def serial_handler(self):
        line = self.comm.readline()     # read the serial input line and set it equal to line
        print 'line'
        print line
        if line.strip() == "RELEASED and CONFIRMED":
            print('um it worked i guess')
            self.read_released(line)
        elif line.strip() == "Response Not Received":  # this is only for status updates not received
            print('not received')
        elif line.strip().__len__() > 5 and line.strip()[0:5] == "Range":
            self.time_since_last = 0
            self.lcm_status.timestamp = int(time.time()*1e6)
            self.lcm_status.id = 0
            self.lcm_status.range = 0
            self.lc.publish(BENTHOS_RELEASE_STATUS_CHANNEL, self.lcm_status.encode())
            print('probably a range')
            self.read_range(line)
      #  elif line.strip().__len__() > 3 and line.strip()[0:3] == "Unit":
      #      benthos_id = int(line[-2:-1])            
      #      self.lcm_status.id = benthos_id                     # looking for a way to send a confirmation through lcm
        elif line.strip() == "Invalid Command":
            print('invalid command')
        else:
            print('unhandled serial string')

#Unit Address 035
#ARMED              or #RELEASED and CONFIRMED
#Tilt Sensor: 7 degrees
#Battery: 89%

#Range 0 to 35: 19.9 m

#RELEASED and CONFIRMED

    
    def read_range(self, line):
        a = line.split(':')
        benthos_range = float(a[1][1:-2])
        benthos_id = int(a[0][-2:])
        
        if benthos_id == BENTHOS_RELEASE_ID_1:
            self.lcm_status.id = 1
        if benthos_id == BENTHOS_RELEASE_ID_2:
            self.lcm_status.id = 2

        # stuff remaining data
        self.lcm_status.range = benthos_range * SOUND_VELOCITY / 1500.

        # publish data
        self.lcm_status.timestamp = int(time.time()*1e6)
        self.lc.publish(BENTHOS_RELEASE_STATUS_CHANNEL, self.lcm_status.encode())
        print("LCM: Published to %s. Range: %f at %d from %d" % (BENTHOS_RELEASE_STATUS_CHANNEL, self.lcm_status.range, self.lcm_status.timestamp, benthos_id))
    
    def read_released(self, line):
        self.lcm_status.timestamp = int(time.time()*1e6)
        self.lc.publish(BENTHOS_RELEASE_STATUS_CHANNEL, self.lcm_status.encode())
        pass


    def main(self):
        while True:
            recv, send, err = select.select([self.lc.fileno(), self.comm.fileno()], [], [], 1.0)
            if recv.__len__() > 0:
                if self.lc.fileno() in recv:
                    # got a LCM message, deal with it
                    self.lc.handle()
                if self.comm.fileno() in recv:
                    # got a serial message, deal with it
                    self.serial_handler()
            else: # timeout or autoranging
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
                        #print('Waiting for response from %d' %(self.id_ranging_to))
                #print('Timeout.')
            self.time_since_last +=1
            # if id is set to 0 that means change the background color of gui
            # the range will be the background color
            # 0 is nothing is wrong
            # 1 is start to worry
            # 2 is stress out now
            if self.time_since_last >= 600 and self.time_since_last < 605:
                self.lcm_status.timestamp = int(time.time()*1e6)
                self.lcm_status.id = 0
                self.lcm_status.range = 1
                self.lc.publish(BENTHOS_RELEASE_STATUS_CHANNEL, self.lcm_status.encode())
            elif self.time_since_last >= 1200 and self.time_since_last < 1205:
                self.lcm_status.timestamp = int(time.time()*1e6)
                self.lcm_status.id = 0
                self.lcm_status.range = 2
                self.lc.publish(BENTHOS_RELEASE_STATUS_CHANNEL, self.lcm_status.encode())

lc = lcm.LCM()
benthos_release_obj  = benthos_release(lc)
benthos_release_obj.main()

# AT%R^<ID>,<release code>[,wakeup]
# AT%RA<ID>,nn,w Activate
# AT%RH<id> Hibernate
# AT%RL<id> Lock
# AT%RU<id>,<Serial No.> UN Lock
# AT%P Get Parameters from R Series Release
# AT%RD<id>Range with depth
# AT%RS
# AT%RR<id> Range
# AT%RT<ID> Check Status of Release
