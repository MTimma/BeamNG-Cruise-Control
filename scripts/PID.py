import os
from time import sleep
import numpy as np
import tkinter as tk

from tkinter import *
from tkinter import messagebox
from tkinter import ttk
import logging
import datetime


from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

from beamngpy import BeamNGpy, Scenario, Vehicle, setup_logging
from beamngpy.sensors import Electrics

class WidgetLogger(logging.Handler):
    def __init__(self, widget):
        logging.Handler.__init__(self)
        self.widget = widget

    def emit(self, record):
        # Append message (record) to the widget
        self.widget.insert(INSERT, record + '\n')
        self.widget.see("end")
        self.widget.pack()
        

##class CruiseControl(object):
##    def __init__(self, interval=1):
##            """ Constructor
##            :type interval: int
##            :param interval: Check interval, in seconds
##            """
##            self.interval = interval
##
##            thread = threading.Thread(target=self.run, args=())
##            thread.daemon = True                            # Daemonize thread
##            thread.start()                                  # Start the execution
##            
##            TARGET = 16;
##            
##
##    
##
##    def run(self):
##        while True:
##            t=0;
##        

def main():

        

    
        # PID parameters
        KP = 0.8 #0.4, 0.6, 0.8
        KI = 0.02
        KD = 0.01
        E = 0;
        windup = 5;
        error_previous = 0
        
        # Logging in separate window
        root = Tk()
        
        T = Text(root)
        logging = WidgetLogger(T)
        root.update_idletasks()
        root.update()

        
        # BeamNGPy setup
        
        # Instantiate BeamNGpy instance running the simulator from the given path,
        
        if ('BNG_HOME' in os.environ):
            bng = BeamNGpy('localhost', 64256, home=os.environ['BNG_HOME'])
        else:
            print ("WARNING no BNG_HOME is set! Make sure to set it as environment variable to research\trunk\ (or hardcode it in the script)")
        # Create a scenario in west_coast_usa called 'example'
        scenario = Scenario('GridMap', 'example')
        
        # Create an ETK800 with the licence plate 'PID'
        vehicle = Vehicle('ego_vehicle', model='etk800', licence='PID')
        
        electrics = Electrics()
        vehicle.attach_sensor('electrics',electrics) #Cars electric system
        
        # Add it to our scenario at this position and rotation
        # scenario.add_vehicle(vehicle, pos=(-717, 101, 118), rot=(0, 0, 45)) 
        scenario.add_vehicle(vehicle, pos=(19.1315, 122.978,  0.163609), rot=(0, 0, 0)) #spawn point for maps GridMap, GridMap2

        # Second vehicle for ACC

        # AI_vehicle = Vehicle('ego_vehicle', model='etk800', licence='AI')
        # AI_electrics = Electrics()
        # vehicle.attach_sensor('electrics',AI_electrics) #AI car electric system 
        # scenario.add_vehicle(vehicle, pos=(-717, 110, 118), rot=(0, 0, 45))
        
        # Place files defining our scenario for the simulator to read
        scenario.make(bng)

        # Launch BeamNG.research
        
        bng.open()
        
        # Load and start our scenario

        bng.load_scenario(scenario)

        # Read target speed first time
        
        s = input('Set Speed (m/s)--> ')
        TARGET = int(s)
        t = input('Set Test Time (sec) --> ')
        TARGET_TIME = int(t)
        bng.start_scenario()
        sensors = bng.poll_sensors(vehicle)

        # Write logs to a file
        
        with open('testLog.txt', 'w+') as f:
            f.write('{0:.6f}\t{1}\t{2:.6f}\t{3}.{4:06d}'.format(0, TARGET, E, 0,0) + '\n')

        
        start_time = datetime.datetime.now()

        #vehicle.ai_set_mode('span')
        #vehicle.ai_set_speed(16,mode='set')
        
        # Print all available "electrics" sensors
        #for i in sensors: 
        #     print (i, sensors[i])
        while True: 

            sensors = bng.poll_sensors(vehicle)
            wheelspeed = sensors['electrics']['values']['wheelspeed']
            
            current_time = datetime.datetime.now()
            elapsed_time = current_time - start_time
            
            #airspeed = sensors['electrics']['values']['airspeed']
            logging.emit('{0:.6f}\t{1}\t{2:.6f}\t{3}:{4:06d}'.format(wheelspeed, TARGET, E, elapsed_time.seconds,elapsed_time.microseconds))
            root.update_idletasks()
            root.update()
            
            with open('testLog.txt','a') as f:
                f.write('{0:.6f}\t{1}\t{2:.6f}\t{3}.{4:06d}'.format(wheelspeed, TARGET, E, elapsed_time.seconds,elapsed_time.microseconds) + '\n')

            
            print ('{0:.6f}\t{1}\t{2:.6f}\t{3}.{4:06d}'.format(wheelspeed, TARGET, E, elapsed_time.seconds,elapsed_time.microseconds)) # 0-40 m/s ?
            
            #PID
            # Error
            error = TARGET - wheelspeed
            # All previous error sum with windup
            # if (error < 0 and E > 5): 
            if (error * error_previous <0):
                if(E>windup):
                    E = windup
                elif(E<-windup):
                    E = -windup
                else:
                    E += error
            else:
                E += error
            #P
            p = KP * error 
            #I
            i = KI * E
            #D
            d = KD * error_previous
            value = p + i + d
            value = max(min(1, value), 0)
            vehicle.control(throttle=value)
            error_previous = error
            if (TARGET_TIME<=elapsed_time.seconds):
                print ("Ending Test")
                bng.close()
                quit()

if __name__ == '__main__':
    main()
