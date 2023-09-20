import select, lcm, string, time, tkMessageBox, Tkinter as tk

from urioce_lcm import benthos_release_command_t, benthos_release_status_t

# some constants
BENTHOS_RELEASE_COMMAND_CHANNEL = "BENTHOS_RELEASE_COMMAND" # the name of the LCM channels
BENTHOS_RELEASE_STATUS_CHANNEL = "BENTHOS_RELEASE_STATUS"

AUTO_RANGE_TIME_DELAY = 60

class benthosGui:
    def __init__(self, lc):
        self.lc = lc
        self.lc.subscribe(BENTHOS_RELEASE_STATUS_CHANNEL, self.status_handle)
        self.cmd_msg = benthos_release_command_t()
        
        self.auto_ranging = False

        self.buildgui()

        self.gui.createfilehandler(self.lc.fileno(), tk.READABLE, self.lc_handle_wrapper)
        self.gui.mainloop()

    def lc_handle_wrapper(self, a, b):
        self.lc.handle()
        

    def buildgui(self):
        self.gui = tk.Tk()
        self.frame = tk.Frame(self.gui)
        self.frame.grid()

        button = tk.Button(self.frame, text="Release 1", command=lambda widget="release1": self.release(1))
        button.grid(row=0, column=0)
        button = tk.Button(self.frame, text="Release 2", command=lambda widget="release2": self.release(2))
        button.grid(row=0, column=1)
        
        button = tk.Button(self.frame, text="Activate 1", command=lambda widget="activate1": self.activate(1))
        button.grid(row=1, column=0)
        button = tk.Button(self.frame, text="Activate 2", command=lambda widget="activate2": self.activate(2))
        button.grid(row=1, column=1)

        button = tk.Button(self.frame, text="Range 1", command=lambda widget="range1": self.range_to(1))
        button.grid(row=2, column=0)
        button = tk.Button(self.frame, text="Range 2", command=lambda widget="range2": self.range_to(2))
        button.grid(row=2, column=1)

        button = tk.Button(self.frame, text="Status 1", command=lambda widget="status1": self.status(1))
        button.grid(row=3, column=0)
        button = tk.Button(self.frame, text="Status 2", command=lambda widget="status2": self.status(2))
        button.grid(row=3, column=1)

        self.auto_range_btn = tk.Checkbutton(self.frame, text="Auto Range", command=lambda widget="auto_ranger": self.auto_range()) #on/off switch?
        self.auto_range_btn.grid(row=4, column=0)
        
        self.time_entry = tk.Entry(self.frame,width=4)
        self.time_entry.grid(row=5, column=0)
        self.time_entry.insert(tk.END,str(AUTO_RANGE_TIME_DELAY))
        button = tk.Button(self.frame, text="Auto Range Delay (s)", command=lambda widget="time_auto_range": self.time_auto_range())
        button.grid(row=5,column=1)

        label = tk.Label(self.frame, text="tx: ")
        label.grid(row=6, column=0)
        self.txlabel = tk.Label(self.frame, text="")
        self.txlabel.grid(row=6, column=1, columnspan=1)

        label = tk.Label(self.frame, text="rx: ")
        label.grid(row=7, column=0)
        self.rxlabel = tk.Label(self.frame, text="")
        self.rxlabel.grid(row=7, column=1, columnspan=1)
        
    

    def release(self, release_id):
        if tkMessageBox.askyesno('Verify Release', 'Are you sure?'):
            if tkMessageBox.askyesno('Be ABSOLUTELY sure', 'Really?'):
                self.cmd_msg.timestamp = int(time.time()*1e6)
                self.cmd_msg.id = release_id
                self.cmd_msg.command = "release"
                self.lc.publish(BENTHOS_RELEASE_COMMAND_CHANNEL, self.cmd_msg.encode())
                print('Releasing %d.' % release_id)
                self.txlabel.config(text='Releasing %d.' % release_id)
            else:
                self.txlabel.config(text='Coward.')
        else:
            self.txlabel.config(text='Literally, be sure next time!')
        
    def activate(self, release_id):
        self.cmd_msg.timestamp = int(time.time()*1e6)
        self.cmd_msg.id = release_id
        self.cmd_msg.command = "activate_release"
        self.lc.publish(BENTHOS_RELEASE_COMMAND_CHANNEL, self.cmd_msg.encode())
        print('Activating %d.' % release_id)
        self.txlabel.config(text='Activating %d.' % release_id)

    def status(self, release_id):
        self.cmd_msg.timestamp = int(time.time()*1e6)
        self.cmd_msg.id = release_id
        self.cmd_msg.command = "confirm_release"
        self.lc.publish(BENTHOS_RELEASE_COMMAND_CHANNEL, self.cmd_msg.encode())
        print('Confirming %d.' % release_id)
        self.txlabel.config(text='Confirming %d.' % release_id)

    def range_to(self, release_id): 
        self.cmd_msg.timestamp = int(time.time()*1e6)
        self.cmd_msg.id = release_id
        self.cmd_msg.command = "poll_range"
        self.lc.publish(BENTHOS_RELEASE_COMMAND_CHANNEL, self.cmd_msg.encode())
        print('Ranging %d.' % release_id)
        self.txlabel.config(text='Ranging %d.' % release_id)

    def auto_range(self): 
        if self.auto_ranging == False:
#            self.auto_range_btn.select()
            self.auto_ranging = True
            print('Automatically Ranging')
            self.txlabel.config(text='Auto Ranging')
        else:
#            self.auto_range_btn.deselect()
            self.auto_ranging = False
            print('Stopped Automatically Ranging')
            self.txlabel.config(text='Stopped Auto Ranging')
        
        self.cmd_msg.timestamp = int(time.time()*1e6)
        self.cmd_msg.id = 0
        self.cmd_msg.command = "auto_range"
        self.lc.publish(BENTHOS_RELEASE_COMMAND_CHANNEL, self.cmd_msg.encode())
        
        
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
        self.cmd_msg.timestamp = int(time.time()*1e6)
        self.cmd_msg.id = 1
        print self.cmd_msg.command[-4:]
        
        self.lc.publish(BENTHOS_RELEASE_COMMAND_CHANNEL, self.cmd_msg.encode())
        print 'Reset Timing to ' + (timing)

    def status_handle(self, channel, data):
        try: msg = benthos_release_status_t.decode(data)
        except ValueError:
            print('Invalid LCM message.')
            return
        #print('Bottle %d fired.' % msg.bottle)
        #self.rxlabel.config(text="Bottle %d fired." % msg.bottle)
        self.rxlabel.config(text="Something about a benthos release?")
        if msg.id == 0:
            self.background_color = msg.range
            # 0 is everything is fine
            # 1 is start to worry
            # 2 is find the thing
            if self.background_color == 0:
                self.frame.config(background='grey')
            elif self.background_color == 1:
                self.frame.config(background='yellow')
            elif self.background_color == 2:
                self.frame.config(background='red')

def main():    
    lc = lcm.LCM()
    benthosgui = benthosGui(lc)

if __name__ == '__main__':
      main()
