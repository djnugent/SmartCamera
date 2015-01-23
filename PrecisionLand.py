#!/usr/bin/python
import math
import time
import cv2
import Queue
import sc_config
from sc_video import sc_video
from sc_dispatcher import sc_dispatcher
from sc_logger import sc_logger
from pl_gui import PrecisionLandGUI as gui
from pl_sim import sim
from pl_util import shift_to_origin, current_milli_time
from CircleDetector import CircleDetector
from vehicle_control import veh_control
from droneapi.lib import VehicleMode, Location, Attitude
from position_vector import PositionVector




'''

if controlling the vehicle:
	if we have a valid image:
		calculate target position
		fly to target
	if we dont have a valid image then wait for vehicle to settle
	if vehicle has settled and there is still no target then climb to increase search area
	if we climbed and didn't find anything then go to last known target Location
	only allow the vehicle to do a climb search so many times

'''




'''
Temporary Changes:
-took out logging ability
-added kill_camera
'''


'''
TODO:
Future:
-have program takeover during landing modes(not guided)
-send warning message to GCS when releasing control
-implement a landing detector and when to release control

Bugs:
-add logic for when the vehicle enters from the side of the landing cylinder and underneath the abort point
	-will cause the vehicle to climb the second it enters the area is the target is not in sight
	-will fix this when the vehicle accepts commands in landing modes
		-will add an intial_approach() method
		-make logic more accepting of land and RTL
-add positive and negative check on parameters
	-inverted on Z axis

Improvements:
-add varaible descent_rate based on distance to target center and altitude
-add better target_detected logic(multiple frames required for a lock)
-add update rate to sc_logger
-fix project file structure
-fix Logging printing to console
'''


class PrecisionLand(object):

	def __init__(self):

		#load config file
		sc_config.config.get_file('Smart_Camera')

		#get camera specs
		self.camera_index = sc_config.config.get_integer('camera','camera_index',0)
		self.camera_width = sc_config.config.get_integer('camera', 'camera_width', 640)
		self.camera_height = sc_config.config.get_integer('camera', 'camera_height', 480)
		self.camera_hfov = sc_config.config.get_float('camera', 'horizontal-fov', 72.42)
		self.camera_vfov = sc_config.config.get_float('camera', 'vertical-fov', 43.3)

		#use simulator
		self.simulator = sc_config.config.get_boolean('simulator','use_simulator',True)

		#how many times to attempt a land before giving up
		self.search_attempts = sc_config.config.get_integer('general','search_attempts', 5)

		#The timeout between losing the target and starting a climb/scan
		self.settle_time = sc_config.config.get_integer('general','settle_time', 1.5)

		#how high to climb in meters to complete a scan routine
		self.climb_altitude = sc_config.config.get_integer('general','climb_altitude', 20)

		#the max horizontal speed sent to autopilot
		self.vel_speed_max = sc_config.config.get_float('general', 'vel_speed_max', 5)

		#P term of the horizontal distance to velocity controller
		self.dist_to_vel = sc_config.config.get_float('general', 'dist_to_vel', 0.15)

		#Descent velocity
		self.descent_rate = sc_config.config.get_float('general','descent_rate', 0.5)

		#roll/pitch value that is considered stable
		self.stable_attitude = sc_config.config.get_float('general', 'stable_attitude', 0.18)

		#Climb rate when executing a search
		self.climb_rate = sc_config.config.get_float('general','climb_rate', -2.0)

		#The height at a climb is started if no target is detected
		self.abort_height = sc_config.config.get_integer('general', 'abort_height', 10)

		#when we have lock on target, only descend if within this radius
		self.descent_radius = sc_config.config.get_float('general', 'descent_radius', 1.0)

		#The height at which we lock the position on xy axis
		self.landing_area_min_alt = sc_config.config.get_integer('general', 'landing_area_min_alt', 1)

		#The radius of the cylinder surrounding the landing pad
		self.landing_area_radius = sc_config.config.get_integer('general', 'landing_area_radius', 20)

		#Whether the landing program can be reset after it is disabled
		self.allow_reset = sc_config.config.get_boolean('general', 'allow_reset', True)

		#Run the program no matter what mode or location; Useful for debug purposes
		self.always_run = sc_config.config.get_boolean('general', 'always_run', True)

		#whether the companion computer has control of the autopilot or not
		self.in_control = False

		#Reset state machine
		self.initialize_landing()

		#debugging: 
		self.kill_camera = False

	def name(self):
		return "Precision_Land"

	def connect(self):
		while(veh_control.is_connected() == False):
			# connect to droneapi
			veh_control.connect(local_connect())
		self.vehicle = veh_control.get_vehicle()

	def run(self):
		sc_logger.text(sc_logger.GENERAL, 'running {0}'.format(self.name()))

		#start a video capture
		if(self.simulator):
			sc_logger.text(sc_logger.GENERAL, 'Using simulator')
			sim.set_target_location(veh_control.get_home())
			#sim.set_target_location(Location(0,0,0))

		else:
			sc_video.start_capture(self.camera_index)


		#create an image processor
		detector = CircleDetector()

		#create a queue for images
		imageQueue = Queue.Queue()

		#create a queue for vehicle info
		vehicleQueue = Queue.Queue()

	 	while veh_control.is_connected():

	 		if(cv2.waitKey(2) == 1113938):
				self.kill_camera =  not self.kill_camera


	 		#Reintialize the landing program when entering a landing mode
	 		if veh_control.controlling_vehicle():
				if not self.in_control:
					if(self.allow_reset):
						sc_logger.text(sc_logger.GENERAL, 'Program initialized to start state')
		 				self.initialize_landing()

				self.in_control = True

			else:
		 		self.in_control = False



	 		#we are in the landing zone or in a landing mode and we are still running the landing program
	 		#just because the program is running does not mean it controls the vehicle
	 		#i.e. in the landing area but not in a landing mode
			if (self.inside_landing_area() or self.in_control or self.always_run) and self.pl_enabled:



		 		#update how often we dispatch a command
		 		sc_dispatcher.calculate_dispatch_schedule()

		 		#get info from autopilot
		 		location = veh_control.get_location()
		 		attitude = veh_control.get_attitude()
		 		'''
		 		#get info from autopilot
		 		location = Location(0.000009,0,location.alt)
		 		attitude = Attitude(0,0,0)
		 		'''

		 		#update simulator
		 		if(self.simulator):
		 			sim.refresh_simulator(location,attitude)

		 		# grab an image
				capStart = current_milli_time()
				frame = self.get_frame()
				capStop = current_milli_time()

				if(self.kill_camera):
					frame[:] = (0,255,0)

		 		
		 		#update capture time
		 		sc_dispatcher.update_capture_time(capStop-capStart)

		 		
				#Process image
				#We schedule the process as opposed to waiting for an available core
				#This brings consistancy and prevents overwriting a dead process before
				#information has been grabbed from the Pipe
				if sc_dispatcher.is_ready():
					#queue the image for later use: displaying image, overlays, recording
					imageQueue.put(frame)
					#queue vehicle info for later use: position processing
					vehicleQueue.put((location,attitude))

					#the function must be run directly from the class
					sc_dispatcher.dispatch(target=detector.analyze_frame, args=(frame,attitude,))
	 			


		 		#retreive results
		 		if sc_dispatcher.is_available():
		 			#results of image processor
		 			results = sc_dispatcher.retreive()
		 			# get image that was passed with the image processor
		 			img = imageQueue.get()
		 			#get vehicle position that was passed with the image processor
		 			location, attitude = vehicleQueue.get()


		 			#overlay gui
		 			rend_Image = gui.add_target_highlights(img, results[3])


		 			#show/record images
		 			sc_logger.image(sc_logger.RAW, img)
		 			sc_logger.image(sc_logger.GUI, rend_Image)

		 			#display/log data
		 			sc_logger.text(sc_logger.ALGORITHM,'RunTime: {0} Center: {1} Distance: {2} Raw Target: {3}'.format(results[0],results[1],results[2],results[3]))
		 			sc_logger.text(sc_logger.AIRCRAFT,attitude)
		 			sc_logger.text(sc_logger.AIRCRAFT,location)

		 			#send commands to autopilot
		 			self.control(results,attitude,location)

		 	else:
		 		if(self.pl_enabled == False):
		 			sc_logger.text(sc_logger.GENERAL, 'Landing disabled')
		 		else:
		 			sc_logger.text(sc_logger.GENERAL, 'Not in landing mode or Landing Area')
		 			




	 	#terminate program
	 	sc_logger.text(sc_logger.GENERAL, 'Vehicle disconnected, Program Terminated')
	 	if(self.simulator == False):
	 		sc_video.stop_capture()


	#initialize_landing - reset the state machine which controls the flow of the landing routine
	def initialize_landing(self):
		
		#how mant times we have attempted landing
		self.attempts = 0

		#Last time in millis since we had a valid target
		self.last_valid_target = 0

		#State variable climbing to scan for the target
		self.climbing = False

		#State variable which determines if this program will continue to run
		self.pl_enabled = True

		#State variable used to represent if autopilot is active
		self.initial_descent = True

		#State variable which represents a know target in landing area
		self.target_detected = False



	#control - how to respond to information captured from camera
	def control(self,target_info,attitude,location):
		#we have control from autopilot
		if veh_control.controlling_vehicle():

			valid_target = False

			now = time.time()

			#detected a target
			if target_info[1] is not None:
				self.target_detected = True
				valid_target = True
				initial_descent = False
				self.last_valid_target = now


			#attempt to use precision landing
			if(self.inside_landing_area() == 1):
				#we have detected a target in landing area
				if(self.target_detected):
					self.climbing = False
					self.initial_descent = False

					#we currently see target
					if(valid_target):
						sc_logger.text(sc_logger.GENERAL, 'Target detected. Moving to target')

						#move to target
						self.move_to_target(target_info,attitude,location)

					#lost target
					else:
						#we have lost the target for more than settle_time
						if(now - self.last_valid_target > self.settle_time):
							self.target_detected = False

						#temporarily lost target,
						#top section of cylinder
						if(veh_control.get_location().alt > self.abort_height):
							sc_logger.text(sc_logger.GENERAL, 'Lost Target: Straight Descent')

							#continue descent
							self.straight_descent()
						else:
							sc_logger.text(sc_logger.GENERAL, 'Lost Target: Holding')

							#neutralize velocity
							veh_control.set_velocity(0,0,0)



				#there is no known target in landing area
				else:

					#currently searching
					if(self.climbing):
						self.climb()

					#not searching, decide next move
					else:
						#top section of cylinder
						if(veh_control.get_location().alt > self.abort_height):
							#initial descent entering cylinder
							if(self.initial_descent):
								sc_logger.text(sc_logger.GENERAL, 'No Target: Initial Descent')

								#give autopilot control
								self.autopilot_land()

							#all other attempts prior to intial target detection
							else:
								sc_logger.text(sc_logger.GENERAL, 'No target: Straight descent')

								#straight descent
								self.straight_descent()

						#lower section of cylinder
						else:
							#we can attempt another land
							if(self.attempts < self.search_attempts):
								self.attempts += 1
								sc_logger.text(sc_logger.GENERAL, 'Climbing to attempt {0}'.format(self.attempts))

								#start climbing
								self.climb()

							#give up and 
							else:
								sc_logger.text(sc_logger.GENERAL, 'Out of attempts: Giving up')

								#give autopilot control
								self.autopilot_land()


			#final descent
			elif(self.inside_landing_area() == -1):
				sc_logger.text(sc_logger.GENERAL, 'In final descent')

				#straight descent
				self.straight_descent()
				self.target_detected = False


			#outside cylinder
			else:
				sc_logger.text(sc_logger.GENERAL, 'Outside landing zone')

				#give autopilot control
				self.autopilot_land()

				self.target_detected = False
				self.initial_descent = True

		#the program is running but the autopilot is in an invalid mode
		else:
			sc_logger.text(sc_logger.GENERAL, 'Not in control of vehicle')


	#release_control - give the autopilot full control and leave it in a stable state
	def release_control(self):
		sc_logger.text(sc_logger.GENERAL, 'Releasing control')

		#put vehicle in stable state
		veh_control.set_velocity(0,0,0)
		#autopilot_land()

		#dont let us take control ever again
		self.pl_enabled = False

		'''
		# if in GUIDED mode switch back to LOITER
		if self.vehicle.mode.name == "GUIDED":
			self.vehicle.mode = VehicleMode("LOITER")
			self.vehicle.flush()
		'''


	#move_to_target - fly aircraft to landing pad
	def move_to_target(self,target_info,attitude,location):
		x,y = target_info[1]

		
		#shift origin to center of image
		x,y = shift_to_origin((x,y),self.camera_width,self.camera_height)
		
		#this is necessary because the simulator is 100% accurate
		if(self.simulator):
			hfov = 48.7
			vfov = 49.7
		else:
			hfov = self.camera_hfov
			vfov = self.camera_vfov


		#stabilize image with vehicle attitude
		x -= (self.camera_width / hfov) * math.degrees(attitude.roll)
		y += (self.camera_height / vfov) * math.degrees(attitude.pitch)


		#convert to distance
		X, Y = self.pixel_point_to_position_xy((x,y),location.alt)

		#convert to world coordinates
		target_heading = math.atan2(Y,X) % (2*math.pi)
		target_heading = (attitude.yaw - target_heading) 
		target_distance = math.sqrt(X**2 + Y**2)


		sc_logger.text(sc_logger.GENERAL, "Distance to target: {0}".format(round(target_distance,2)))
	

		#calculate speed toward target
		speed = target_distance * self.dist_to_vel
		#apply max speed limit
		speed = min(speed,self.vel_speed_max)

		#calculate cartisian speed
		vx = speed * math.sin(target_heading) * -1.0
		vy = speed * math.cos(target_heading) 

		#only descend when on top of target
		if(target_distance > self.descent_radius):
			vz = 0
		else:
			vz = self.descent_rate


		#send velocity commands toward target heading
		veh_control.set_velocity(vx,vy,vz)



	#autopilot_land - Let the autopilot execute its normal landing procedure
	def autopilot_land(self):
		#descend velocity
		veh_control.set_velocity(0,0,self.descent_rate)
		#veh_control.set_velocity(9999,9999,9999)


	#straight_descent - send the vehicle straight down
	def straight_descent(self):
		veh_control.set_velocity(0,0,self.descent_rate)


	#climb - climb to a certain alitude then stop.
	def climb(self):

		if(veh_control.get_location().alt < self.climb_altitude):
			sc_logger.text(sc_logger.GENERAL, 'climbing')
			veh_control.set_velocity(0,0,self.climb_rate)
			self.climbing = True
		else:
			sc_logger.text(sc_logger.GENERAL, 'Reached top of search zone')
			veh_control.set_velocity(0,0,0)
			self.climbing = False


	#inside_landing_area - determine is we are in a landing zone 0 = False, 1 = True, -1 = below the zone
	def inside_landing_area(self):
		
		vehPos = PositionVector.get_from_location(veh_control.get_location())
		landPos = PositionVector.get_from_location(veh_control.get_landing())
		'''
		vehPos = PositionVector.get_from_location(Location(0,0,10))
		landPos = PositionVector.get_from_location(Location(0,0,0))
		'''
		if(PositionVector.get_distance_xy(vehPos,landPos) < self.landing_area_radius):
			#below area
			if(vehPos.z < self.landing_area_min_alt):
				return -1
			#in area
			else:
				return 1
		#outside area
		else:
			return 0



	#get_frame - pull an image from camera or simulator
	def get_frame(self):
		if(self.simulator):
			return sim.get_frame()
		else:
			return sc_video.get_image()


	#pixel_point_to_position_xy - convert position in pixels to position in meters
	#pixel_position - distance in pixel from CENTER of image
	#distance- distance from the camera to the object in  meters 
	def pixel_point_to_position_xy(self,pixel_position,distance):
		thetaX = pixel_position[0] * self.camera_hfov / self.camera_width
		thetaY = pixel_position[1] * self.camera_vfov / self.camera_height
		x = distance * math.tan(math.radians(thetaX))
		y = distance * math.tan(math.radians(thetaY))

		return (x,y)





# if starting from mavproxy
if __name__ == "__builtin__":
	# start precision landing
	strat = PrecisionLand()

	# connect to droneapi
	sc_logger.text(sc_logger.GENERAL, 'Connecting to vehicle...')
	strat.connect()
	sc_logger.text(sc_logger.GENERAL, 'Vehicle connected!')

	# run strategy
	strat.run()








