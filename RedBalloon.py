#!/usr/bin/python
import sc_config
from sc_logger import sc_logger


class RedBalloon(object):

	def __init__(self,args):
		#load config file
		sc_config.config.get_file(self.name())

		#get camera index
		self.camera_index = int(args.camera)

	def name(self):
		return "Red_Balloon_Finder"

	def run(self):
		sc_logger.text(sc_logger.GENERAL, 'running {0}'.format(self.name()))

