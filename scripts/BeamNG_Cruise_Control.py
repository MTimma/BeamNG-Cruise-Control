import os

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




	def calculate_sum(self, windup):
		# mērķa ātrums tiek sasniegts
		if (self.curr * self.prev < 0):
			if (self.sum > windup):
				self.sum = windup
			elif (self.sum < -(windup)):
				self.sum = -(windup)
			else:
				self.sum += self.curr
		else:
			self.sum += self.curr





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
		if "targets" in keys:
			self.targets = kwargs.get("targets")
		else:
			self.targets = [0]
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
		scenario.add_vehicle(self.vehicle, pos=(406.787, 1115.518, 0),
							 rot=(0, 0, 0))

		# Place files defining our scenario for the simulator to read
		scenario.make(self.bng)

		# Launch BeamNG.research

		self.bng.open()

		# Load and start our scenario

		self.bng.load_scenario(scenario)
		self.bng.start_scenario()




	def runTest(self, test_K_values, test_name):
		for test_K_value in test_K_values:
			KP_in, KI_in, KD_in = map(float,test_K_value)

			if KP_in == 0 and KI_in == 0 and KD_in == 0:
				config = Config()
				controller = On_Off_Controller(config=config)
			else:
				config = Config(KP=KP_in,KI=KI_in,KD=KD_in)
				controller = PID_Controller(config=config)
			self.controller.controller=controller
			self.test_name= str(test_name) + "_" + str(KP_in) + "_" + str(KI_in) + "_" + str(KD_in)
			self.runTestForPID()




	def runTestForPID(self):
		for speed in self.targets:
			self.controller.setTarget(speed)
			self.controller.setTime(float(17))
			self.runTestOfType("up_{0}_".format(speed), (406.787, 700.517, 0.214829), (0, 0, 0))
			self.runTestOfType("straight_{0}_".format(speed), (406.787, 715.517, 0.214829), (0, 0, 180))
			self.runTestOfType("down_{0}_".format(speed), (406.787, 304.896, 100.211), (0, 0, 180))

	def runTestOfType(self, type, pos, rot):
		i = 1
		while i<=self.testing_times:
			self.controller.newLogFile(self.test_name + "_" + str(type) +"_" + str(i) + ".txt")
			self.bng.restart_scenario()
			self.bng.teleport_vehicle(self.vehicle, pos=pos, rot=rot)
			self.run()
			i+=1

	def run(self):
		self.controller.start()
		while self.controller.ended==False:
			start_time = datetime.datetime.now()
			sensors = self.bng.poll_sensors(self.vehicle)
			wheel_speed = sensors['electrics']['values']['wheelspeed']
			value = self.controller.calculate_speed_with_logs(wheel_speed)
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

	def close(self):
		self.bng.close()



class Cruise_Controller_Test():
	def __init__(self, **kwargs):
		keys = kwargs.keys()
		if "controller" in keys:
			self.controller = kwargs.get("controller")
		else:
			self.controller = PID_Controller()
		if "test_time" in keys:
			self.time = kwargs.get("test_time")
		else:
			self.time = 0
		self.started = False
		self.started_L = False
		self.ended = False
		self.target_speed = 0

	def setTime(self,time):
		self.time = time

	def setTarget(self, speed):
		self.target_speed=speed
		self.controller.change_target(speed)

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
			f.write(formatLog(datetime.timedelta(seconds=0, microseconds=0), 0, self.target_speed) + '\n')

	def startTimer(self):
		self.started_T = True
		self.start_time = datetime.datetime.now()

	def calculateElapsed(self):
		current_time = datetime.datetime.now()
		elapsed_time = current_time - self.start_time
		return elapsed_time

	def logSpeed(self, wheel_speed, elapsed_time):
		with open(self.filename, 'a') as f:
			f.write(formatLog(elapsed_time, wheel_speed, self.target_speed) + '\n')
		print(formatLog(elapsed_time, wheel_speed, self.target_speed))

	def calculate_speed_with_logs(self, wheel_speed):
		if self.started and self.ended==False:
			elapsed_time = self.calculateElapsed()
			end_time = self.time
			seconds_error = elapsed_time.seconds - end_time
			if seconds_error >= 0:
				print("Timer has ended")
				self.ended=True
				return 0
			self.logSpeed(wheel_speed, elapsed_time)
			return self.controller.calculate_throttle(wheel_speed)
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

	def change_target(self, target):
		self.config.target = target

	def calculate_throttle(self, wheel_speed):
		# Error
		self.error.new(self.config.target - wheel_speed)
		self.error.calculate_sum(self.config.windup)
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
	def change_target(self, target):
		self.config.target = target

	def calculate_throttle(self, wheel_speed):
		if wheel_speed > self.config.target:
			return -1
		else:
			return 1




def createBeamNG(target_speeds, test_count):
	s = list(map(float, target_speeds))
	test_count = int(test_count)
	controller = PID_Controller()
	test_controller = Cruise_Controller_Test(controller=controller)
	test = BeamNG_Cruise_Controller_Test(test_controller=test_controller, testing_times=test_count, targets=s)
	test.setup_bngServer()
	test.setup_BeamNG()
	return test




def main():
		#If multiple Target Speeds are set, same amount of Test Times have to be specified

		s_in = input('Set Speed (m/s)-->')
		s_list = s_in.split(" ")
		s = list(map(float, s_list))
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
		test_controller = Cruise_Controller_Test(controller=controller)
		test = BeamNG_Cruise_Controller_Test(test_controller=test_controller, testing_times=test_count,
		test_name="BeamNG_test_{0}".format(test_name_in), targets=s)
		test.run()




def formatLog(elapsed_time, wheel_speed, target):
	return '{2}.{3:06d}\t{0:.6f}\t{1:.6f}'.format(wheel_speed, target, elapsed_time.seconds, elapsed_time.microseconds)



if __name__ == '__main__':
	main()
