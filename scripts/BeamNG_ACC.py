import os
import time
import math
import logging
import datetime
import BeamNG_Cruise_Control
import threading

from beamngpy import BeamNGpy, Scenario, Vehicle, setup_logging
from beamngpy.sensors import Electrics

#Main logic is in method `run` @79
CC_P = 0.8
CC_I = 0.05
CC_D = 0.05
ACC_CAR_SPEED = 20
FRONT_CAR_SPEED = 15
WANTED_DISTANCE = 30
ACC_CAR_POS=(406.787, 700.517, 0)
FRONT_CAR_POS=(406.78692626953125,750.516845703125,0)



class BeamNG_ACC_Test():
	def __init__(self, **kwargs):
		keys = kwargs.keys()
		if "test_controller" in keys:
			self.controller = kwargs.get("test_controller")
		else:
			self.controller = ACC_Test()
		config = BeamNG_Cruise_Control.Config(KP=CC_P, KI=CC_I, KD=CC_D, target=ACC_CAR_SPEED)
		config2 = BeamNG_Cruise_Control.Config(KP=CC_P, KI=CC_I, KD=CC_D, target=FRONT_CAR_SPEED)
		self.PID = BeamNG_Cruise_Control.PID_Controller(config=config)
		self.PID2 = BeamNG_Cruise_Control.PID_Controller(config=config2)
	def setup_bngServer(self):
		# Instantiate BeamNGpy instance running the simulator from the given path,
		if ('BNG_HOME' in os.environ):
			self.bng = BeamNGpy('localhost', 64256, home=os.environ['BNG_HOME'])
		else:
			print(
				"WARNING no BNG_HOME is set! Make sure to set BeamNG.research path to research\trunk\ as environment variable (or write the path in the script by yourself)")
			self.bng = BeamNGpy('localhost', 64256, home='C:\\Users\\Ingars\\Beamng-unlimited\\trunk')
	def setup_BeamNG(self):
		# Create a scenario in test map
		self.scenario = Scenario('cruise-control', 'example')

		# Create an ETK800 with the licence plate 'PID'
		self.vehicle = Vehicle('ACC', model='etk800', licence='ACC')

		electrics = Electrics()
		self.vehicle.attach_sensor('electrics', electrics)  # Cars electric system

		# Add it to our scenario at this position and rotation
		self.scenario.add_vehicle(self.vehicle, pos=ACC_CAR_POS,
							 rot=(0, 0, 0))

		self.vehicle2 = Vehicle('FRONT', model='etk800', licence='FRONT')
		self.vehicle2.attach_sensor('electrics', electrics)  # Cars electric system
		self.scenario.add_vehicle(self.vehicle2, pos=FRONT_CAR_POS, rot=(0, 0, 180) )
		# Place files defining our scenario for the simulator to read
		self.scenario.make(self.bng)

		# Launch BeamNG.research

		self.bng.open()

		# Load and start our scenario

		self.bng.load_scenario(self.scenario)
		self.bng.start_scenario()




	def runTest(self):
		self.bng.restart_scenario()
		self.bng.teleport_vehicle(self.vehicle, pos=ACC_CAR_POS, rot=(0, 0, 180))
		self.run()

	def run(self):
		self.setupTest()
		wanted_t = ACC_CAR_SPEED
		wanted_d = WANTED_DISTANCE

		soft_brake_d = wanted_d - (wanted_d/2)

		wanted_d_buffer = wanted_d - (wanted_d / 5)
		# Wanted distance has a buffer of 25%, to compensate holding same constant speed at bigger distance

		deacc_d = wanted_d_buffer + 10
		# +10 meters is buffer for starting to decelerate towards front car speed and wanted distance.
		# the buffer should be calculated dynamically instead to compensate for different speeds
		while self.controller.ended==False:
			start_time = datetime.datetime.now()
			sensors = self.bng.poll_sensors(self.vehicle)
			position = self.vehicle.state['pos']
			position2 = self.vehicle2.state['pos']
			sensors2 = self.bng.poll_sensors(
				self.vehicle2)  # replace with self.vehicle.update_vehicle when not using pre-generated coordinates of vehicle to follow

			wheel_speed = sensors['electrics']['values']['wheelspeed']

			distance = self.controller.setDistance(position, position2)

			front_car_speed = self.controller.getFrontCarSpeed(wheel_speed)
			curr_target = self.PID.get_target()
			print("distance: " + str(distance))
			#FORMULA USED for deacceleration = front_car_speed^2 = wheel_speed^2 + (distance-20)*2*a
			#d = (front_car_speed - wheel_speed) / (-0.3 * 2) # 0.3 deacceleration? distance for reducing speed to front cars speed with -0.3 acceleration
			#print("d: " + str(d))
			if  distance < 5: # 5 is manually set as last allowed distance between both cars, will not work good if ACC car is driving too fast. Would be better to calculate it as braking distance.
				print("brake1")
				self.vehicle.control(brake=1)
				self.vehicle.control(throttle=0)
			elif distance < soft_brake_d:
				print("brake1")
				self.vehicle.control(brake=0.1) #Softness of brake could also be calculated dynamically to compensate different speeds
				self.vehicle.control(throttle=0)
			else:
				if distance <= wanted_d_buffer:
					print("wanted")
					if front_car_speed > curr_target + 3.5  or front_car_speed < curr_target - 2: #or front_car_speed < curr_target - 1   // calibrate 3.5 and 2
						self.PID.change_target(max(front_car_speed, 0))
				elif distance <= deacc_d:
					a = (front_car_speed - wheel_speed) / ((distance - wanted_d_buffer) * 2)
					print("a:" + str(a))
					self.PID.change_target(a + wheel_speed)
				elif curr_target != wanted_t:
					self.PID.change_target(wanted_t)
				print("throttle1")
				value = self.PID.calculate_throttle(wheel_speed)
				value = min(1, value)
				self.vehicle.control(brake=0)
				self.vehicle.control(throttle=value)
			#PID for front car
			wheel_speed2 = sensors2['electrics']['values']['wheelspeed']
			# print("real front:" + str(wheel_speed2))
			value2 = self.PID2.calculate_throttle(wheel_speed2)
			value2 = min(1, value2)
			self.vehicle2.control(brake=0)
			self.vehicle2.control(throttle=value2)


			elapsed_time = datetime.datetime.now() - start_time
			while (elapsed_time.total_seconds() * 1000) < 100:
				elapsed_time = datetime.datetime.now() - start_time
			elapsed_total = self.controller.calculateElapsed()
			# Change front car speed after 10 seconds and after 20 seconds
			if elapsed_total.total_seconds() > float(10):
				self.PID2.change_target(20)
			if elapsed_total.total_seconds() > float(20):
				self.PID2.change_target(10)
		print("Ending Test")
	def close(self):
		self.bng.close()
	def setupTest(self):
		self.vehicle.update_vehicle()
		self.controller.last_position = self.vehicle.state['pos']
		self.controller.start()

class ACC_Test():
	def __init__(self):
		self.started = False
		self.ended = False
		self.target_speed = 0
		self.last_distance_timestamp = (0, datetime.timedelta(seconds=0))
		self.current_distance_timestamp = (0, datetime.timedelta(seconds=0))
		self.last_position = (0, 0, 0)
		self.last_distance_driven = 0

	def setTime(self,time):
		self.time = time

	def newLogFile(self, filename):
		self.filename = filename

	def start(self):
		self.started = True
		self.ended = False
		self.target_index = 0
		self.startTimer()

	def startTimer(self):
		self.started_T = True
		self.start_time = datetime.datetime.now()

	def calculateElapsed(self):
		current_time = datetime.datetime.now()
		elapsed_time = current_time - self.start_time
		return elapsed_time

	def getFrontCarSpeed(self, speed):
		#print("last dist driven=" + str(self.last_distance_driven))
		#print("current dist=" + str(self.current_distance_timestamp[0]))
		#print("last dist=" + str(self.last_distance_timestamp[0]))
		time_driven = self.current_distance_timestamp[1].total_seconds() -  self.last_distance_timestamp[1].total_seconds()
		front_distance_driven = self.current_distance_timestamp[0] + time_driven * speed  - self.last_distance_timestamp[0]
		#print("front car dist=" + str(front_distance_driven))
		print("front car time=" + str(time_driven))
		front_car_speed = front_distance_driven / time_driven
		return front_car_speed

	def setDistance(self, position, position2):
		current_distance = self.calculateDistance(position, position2)
		self.last_distance_timestamp = self.current_distance_timestamp
		self.current_distance_timestamp = (current_distance, self.calculateElapsed())
		return current_distance

	def calculateDistance(self, position, position2):
		position_front = position2
		#print("position_front:" + '(' +  ','.join(map(str, position_front)) + ')')
		#print("position:" + '(' + ','.join(map(str, position)) + ')')
		dx = position_front[0] - position[0]
		#print("dx:" + str(dx))
		dy = position_front[1] - position[1]
		#print("dy:" + str(dy))
		distance = math.sqrt(dx**2 + dy**2)

		if dy<0: #straight driving in test on y, if dy is <0 then the car runs into front vehicle
			distance = -1 *  distance
		#print("distance:" + str(distance))
		return distance

def createBeamNG():
	test_controller = ACC_Test()
	test = BeamNG_ACC_Test(test_controller=test_controller)
	test.setup_bngServer()
	test.setup_BeamNG()
	return test

if __name__ == '__main__':
	main()
