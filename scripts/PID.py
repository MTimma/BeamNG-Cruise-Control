import os
from time import sleep
import numpy as np
import tkinter as tk

import logging
import datetime

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

class Config():
	def __init__(self, **kwargs):
		keys = kwargs.keys()
		if "KP" in keys:
			self.KP = kwargs.get("KP")
		else:
			self.KP = 0
		if "KI" in keys:
			self.KI = kwargs.get("KI")
		else:
			self.KI = 0
		if "KD" in keys:
			self.KD = kwargs.get("KD")
		else:
			self.KD = 0
		if "windup" in keys:
			self.windup = kwargs.get("windup")
		else:
			self.windup = 5
		if "target" in keys:
			self.target = kwargs.get("target")
		else:
			self.target = 0




class Error():
	def __init__(self):
		self.sum = 0
		self.prev = 0
		self.curr = 0
	def new(self, new):
		self.prev = self.curr
		self.curr = new
	def calculateSum(self, windup):
		if (self.curr * self.prev < 0): # Change from positive to negative (or neg to pos)
			if (self.sum > windup):
				self.sum = windup
			elif (self.sum < -(windup)):
				self.sum = -(windup)
			else:
				self.sum += self.sum
		else:
			self.sum += self.sum

class BeamNG_Cruise_Controller_Test():
	def __init__(self, **kwargs):
		keys = kwargs.keys()
		if "test_controller" in keys:
			self.controller = kwargs.get("test_controller")
		else:
			self.controller = PID_Controller_Test()
		if "testing_times" in keys:
			self.testing_times = kwargs.get("testing_times")
		else:
			self.testing_times = 1
		if "test_name" in keys:
			self.test_name = kwargs.get("test_name")
		else:
			self.test_name = "test"
	def setup_bngServer(self):
		# Instantiate BeamNGpy instance running the simulator from the given path,
		if ('BNG_HOME' in os.environ):
			self.bng = BeamNGpy('localhost', 64256, home=os.environ['BNG_HOME'])
		else:
			print(
				"WARNING no BNG_HOME is set! Make sure to set path to research\trunk\ as environment variable (or write the path in the script by yourself)")
			self.bng = BeamNGpy('localhost', 64256, home='C:\\Users\\Ingars\\SVN\\trunk')
	def setup_BeamNG(self):
		

		# Create a scenario in test map
		scenario = Scenario('cruise-control', 'example')

		# Create an ETK800 with the licence plate 'PID'
		self.vehicle = Vehicle('ego_vehicle', model='etk800', licence='PID')

		electrics = Electrics()
		self.vehicle.attach_sensor('electrics', electrics)  # Cars electric system

		# Add it to our scenario at this position and rotation
		# scenario.add_vehicle(vehicle, pos=(-717, 101, 118), rot=(0, 0, 45))
		scenario.add_vehicle(self.vehicle, pos=(406.787, 815.518, 0.214847),
							 rot=(1, 0, 0))	 # spawn point for maps GridMap, GridMap2

		# Place files defining our scenario for the simulator to read
		scenario.make(self.bng)

		# Launch BeamNG.research

		self.bng.open()

		# Load and start our scenario

		self.bng.load_scenario(scenario)

	def run(self):
		i = 1
		while i<=self.testing_times:
			self.runTestNr(i)
			i+=1

	def runTestNr(self, nr):
		self.setup_bngServer()
		self.setup_BeamNG()
		self.controller.newLogFile(self.test_name + "_" + str(nr) + ".txt")
		self.bng.start_scenario()
		self.controller.start()
		while self.controller.ended==False:
			start_time = datetime.datetime.now()
			sensors = self.bng.poll_sensors(self.vehicle)
			wheelspeed = sensors['electrics']['values']['wheelspeed']
			value = self.controller.calculate_speed_with_logs(wheelspeed)
			if value <= 0:
				value = max(-1, value -0.1) * -1
				self.vehicle.control(brake=value)
				self.vehicle.control(throttle=0)
			else:
				value = min(1, value)
				self.vehicle.control(brake=0)
				self.vehicle.control(throttle=value)
			elapsed_time = datetime.datetime.now() - start_time
			while (elapsed_time.total_seconds() * 1000) < 100:
				elapsed_time = datetime.datetime.now() - start_time
		print("Ending Test")
		self.bng.close()


class Cruise_Controller_Test():
	def __init__(self, **kwargs):
		keys = kwargs.keys()
		if "controller" in keys:
			self.controller = kwargs.get("controller")
		else:
			self.controller = PID_Controller()
		if "filename" in keys:
			self.filename = kwargs.get("filename")
		else:
			self.filename = 'testLog.txt'
		if "targets" in keys:
			self.targets = kwargs.get("targets")
		else:
			print("WARNING, no target speeds set")
			self.targets = [0]
		if "target_times" in keys:
			self.target_times = kwargs.get("target_times")
		else:
			print("WARNING, no target times set")
			self.target_times = [0]
		self.started = False
		self.started_T = False
		self.started_L = False
		self.target_index = 0
		self.ended = False
		self.controller.config.target = self.targets[self.target_index]

	def newLogFile(self, filename):
		self.filename = filename

	def start(self):
		self.started = True
		self.ended = False
		self.target_index = 0
		self.startLogging()
		self.startTimer()

	def startLogging(self):
		self.started_L = True
		with open(self.filename, 'w+') as f:
			f.write(formatLog(datetime.timedelta(seconds=0, microseconds=0), 0, self.targets[self.target_index]) + '\n')

	def startTimer(self):
		self.started_T = True
		self.start_time = datetime.datetime.now()

	def calculateElapsed(self):
		current_time = datetime.datetime.now()
		elapsed_time = current_time - self.start_time
		return elapsed_time

	def logSpeed(self, wheel_speed, elapsed_time):
		with open(self.filename, 'a') as f:
			f.write(formatLog(elapsed_time, wheel_speed, self.targets[self.target_index]) + '\n')
		print(formatLog(elapsed_time, wheel_speed, self.targets[self.target_index]))

	def calculate_speed_with_logs(self, wheel_speed):
		if self.started and self.ended==False:
			elapsed_time = self.calculateElapsed()
			end_time = sum(self.target_times[0:self.target_index + 1])
			seconds_error = elapsed_time.seconds - end_time
			if seconds_error >= 0:
				if self.target_index + 1 == len(self.target_times):
					self.ended = True
					self.started = False
					print("Timer has ended")
					return 0
				else:
					self.target_index += 1
					self.controller.config.target = self.targets[self.target_index]
					print("Next target speed = {0}".format(self.targets[self.target_index]))
					print("Timer was late by {0}.{1:06d} sec".format(seconds_error, elapsed_time.microseconds))
			self.logSpeed(wheel_speed, elapsed_time)
			return self.controller.calculate_speed(wheel_speed)
		else:
			print("Ended or not started yet")
			return 0

class PID_Controller():
	def __init__(self, **kwargs):
		keys = kwargs.keys()
		if "config" in keys:
			self.config = kwargs.get("config")
		else:
			self.config = Config()
		if "error" in keys:
			self.error = kwargs.get("error")
		else:
			self.error = Error()

	def changeTarget(self, target):
		self.config.target = target

	def calculate_speed(self, wheel_speed):
		# Error
		self.error.new(self.config.target - wheel_speed)
		self.error.calculateSum(self.config.windup)
		# P
		p = self.config.KP * self.error.curr
		# I
		i = self.config.KI * self.error.sum
		# D
		d = self.config.KD * self.error.prev
		return p + i + d

class On_Off_Controller():
	def __init__(self, **kwargs):
		keys = kwargs.keys()
		if "config" in keys:
			self.config = kwargs.get("config")
		else:
			self.config = Config()
		if "error" in keys:
			self.error = kwargs.get("error")
		else:
			self.error = Error()
	def changeTarget(self, target):
		self.config.target = target

	def calculate_speed(self, wheel_speed):
		if wheel_speed > self.config.target:
			return 0
		else:
			return 1

def main():
		#If multiple Target Speeds are set, same amount of Test Times have to be specified

		s_in = input('Set Speed (m/s)--> ')
		s_list = s_in.split(",")
		s = list(map(int, s_list))

		t_in = input('Set Test Time (sec) --> ')
		t_list = t_in.split(",")
		t = list(map(int, t_list))
		KP_in = float(input('Enter KP --> '))

		KI_in = float(input('Enter KI --> '))
		KD_in = float(input('Enter KD --> '))
		test_count = int(input('How many tests to perform? --> '))
		test_name_in = input('Test name for log files --> ')
		if KP_in == 0 and KI_in == 0 and KD_in == 0:
			config = Config()
			controller = On_Off_Controller(config=config)
		else:
			config = Config(KP=KP_in,KI=KI_in,KD=KD_in)
			controller = PID_Controller(config=config)
		test_controller = Cruise_Controller_Test(controller=controller, targets=s, target_times=t)
		test = BeamNG_Cruise_Controller_Test(test_controller=test_controller, testing_times=test_count,
		test_name="BeamNG_test_{0}".format(test_name_in))
		test.run()


def formatLog(elapsed_time, wheel_speed, target):
	return '{2}.{3:06d}\t{0:.6f}\t{1:.6f}'.format(wheel_speed, target, elapsed_time.seconds, elapsed_time.microseconds)


if __name__ == '__main__':
	main()
