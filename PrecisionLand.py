import time
import cv2
import Queue
import sc_config
from pl_gui import PrecisionLandGUI as gui
from sc_video import sc_video
from sc_dispatcher import sc_dispatcher
from sc_logger import sc_logger
from CircleDetector import CircleDetector
from vehicle_control import veh_control
from pymavlink import mavutil
from droneapi.lib import VehicleMode, Location

current_milli_time = lambda: int(time.time() * 1000)



class PrecisionLand(object):

	def __init__(self,args):

		#load config file
		sc_config.config.get_file(self.name())

		#get camera index
		self.camera_index = int(args.camera)


	def name(self):
		return "Precision_Land"

	def connect(self):
		# connect to droneapi
		veh_control.connect(local_connect())

	def run(self):
		sc_logger.text(sc_logger.GENERAL, 'running {0}'.format(self.name()))

		#start a video capture
		sc_video.start_capture(self.camera_index)

		#create an image processor
		detector = CircleDetector()

		#create a queue for images
		imageQueue = Queue.Queue()

	 	while True:

	 		#update how often we dispatch a command
	 		sc_dispatcher.calculate_dispatch_schedule()

	 		# grab an image
			capStart = current_milli_time()
			frame = sc_video.get_image()
			capStop = current_milli_time()

	 		
	 		#update capture time
	 		sc_dispatcher.update_capture_time(capStop-capStart)


	 		
			#Process image
			#We schedule the process as opposed to waiting for an available core
			#This brings consistancy and prevents overwriting a dead process before
			#information has been grabbed from the Pipe
			if sc_dispatcher.is_ready():
				#queue the image for later use: displaying image, overlays, recording
				imageQueue.put(frame)

				#the function must be run directly from the class
				sc_dispatcher.dispatch(target=detector.analyze_frame, args=(frame,None,))
	 			


	 		#retreive results
	 		if sc_dispatcher.is_available():
	 			#results of image processor
	 			results = sc_dispatcher.retreive()
	 			# get image that was passed with the image processor
	 			img = imageQueue.get()
	 			#overlay gui
	 			rend_Image = gui.add_target_highlights(img, results[3])
	 			#show/record images
	 			sc_logger.image(sc_logger.RAW, img)
	 			sc_logger.image(sc_logger.GUI, rend_Image)

			# update vehicle control
			veh_control.run()

# if starting from command line
if __name__ == "__main__":
	class arg(object):
		camera = 0

	arg = arg()

	strat = PrecisionLand(arg)
	strat.run()

# if starting from simulator
elif __name__ == "__builtin__":
	class arg(object):
		camera = 0

	arg = arg()

	# start precision landing
	strat = PrecisionLand(arg)

	# connect to droneapi
	strat.connect()

	# run strategy
	strat.run()