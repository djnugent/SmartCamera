import logging
import time
import argparse
import os
from pymavlink import mavutil
from droneapi.lib import VehicleMode, Location
import sc_config
from sc_logger import sc_logger
#startegies
from PrecisionLand import PrecisionLand
from RedBalloon import RedBalloon
from OpticalFlow import OpticalFlow



#parse arguments
parser = argparse.ArgumentParser(description="Use image processing to peform visual navigation")
#optional arguments
parser.add_argument('-S', '--Strategy', action="store", default='land', type=str,
 					choices=['land','redballoon','opticalflow'], 
 					help='Land: vision assisted landing \n redballoon: redballoon finder for AVC 2014 \n opticalflow: Optical flow using webcam')
parser.add_argument('-c', '--camera', default=0, help="Select the camera index for opencv")
parser.add_argument('-i', '--input', default=False, help='use a video filename as an input instead of a webcam')
parser.add_argument('-f', '--file', default='Smart_Camera.cnf', help='load a config file other than the default')

args = parser.parse_args()



#run a strategy
Strategy = None
if args.Strategy == 'land':
	Strategy = PrecisionLand(args)
if args.Strategy == 'redballoon':
	Strategy = RedBalloon(args)
if args.Strategy == 'opticalflow':
	Strategy = OpticalFlow(args)


#configure a logger
sc_logger.set_name(Strategy.name())


sc_logger.text(sc_logger.GENERAL, 'Starting {0}'.format(Strategy.name()))

Strategy.run()



