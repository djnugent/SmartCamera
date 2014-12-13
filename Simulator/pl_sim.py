import cv2
import numpy as np
from matplotlib import pyplot as plt
import math
from droneapi.lib import Location

'''
TODO 
Finish main method
Finish get_frame method
	-Position coversions
	-frameRate
convert as much as possible to PositionVector Type and Location Type(from droneapi.lib)
	-clean up tuples
organize helper methods. Possibly put in a seperate class
Intergrate config file


Long term enhancements: 
Fix position_to_pixel. Do it properly with vector based math
Add in textured/tiled background
dynamic exposure levels / frame rate 
Add google earth as background
'''



class PrecisionLandSimulator():


	def __init__(self):
		self.backgroundColor = (0,0,0)

	#load_target- load an image to simulate the target. Enter the actaul target size in meters(assuming the target is square)
	#might change img to file name instead
	def load_target(self,img, actualSize):
		self.target = img
		self.target_width = img.shape[1]
		self.target_height = img.shape[0]


		self.actualSize = actualSize
		#scaling factor for real dimensions to simultor pixels
		self.pixels_per_meter = (self.target_height + self.target_width) / (2.0 * actualSize) 



	#set_target_location- give the target a gps location
	def set_target_location(self, latitude, longitude, altitude):
		self.targetLat = latitude
		self.targetLon = longitude
		self.targetAlt = altitude

	#define_camera- set camera parameters used to simulate an image
	def define_camera(self, camera_width, camera_height, vfov, hfov, frameRate):
		self.camera_width = camera_width
		self.camera_height = camera_height
		self.camera_vfov = vfov
		self.camera_hfov = hfov
		self.camera_frameRate = frameRate

	#refresh_simulator - update vehicle position info necessary to simulate an image 
	def refresh_simulator(self, vehicleLocation, vehicleAttitude):
		#get gps location of vehicle
		#self.vehicleLocation = PositionVector.get_from_location(PositionVector.get_home_location())
		self.vehicleLat = vehicleLocation[0]
		self.vehicleLon = vehicleLocation[1]
		self.vehicleAlt = vehicleLocation[2]

		#get attitude of vehicle
		self.vehicleRoll  = vehicleAttitude[0]
		self.vehiclePitch = vehicleAttitude[1]
		self.vehicleYaw = vehicleAttitude[2]



	#get_frame - return a frame from the simulated camera at the specified capture rate 
	def get_frame(self):
		#distance bewteen camera and target in meters
		meters_dX = 
		meters_dY = 
		meters_dZ = 
		#distance bewtween camera and targer in pixels
		dX = meters_dX * pixels_per_meter
		dY = meters_dY * pixels_per_meter
		dZ = meters_dZ * pixels_per_meter


		sim = self.simulate_target(self.target,dX,dY,dZ,self.vehicleRoll,self.vehiclePitch,self.vehicleYaw)
		return sim


	def main(self):
		'''
		img = cv2.imread('targetF.jpg')
		self.load_target(img,1.0)
		self.define_camera(640,480,62,70,30)
		self.set_target_location()

		while True:
			self.refresh_simulator()

			frame = self.getFrame()
		'''


		################################# HELPER METHODS ####################
	#cartisian_to_polar- convert to polar coordinates
	#Can be cleaned up. A bit rough
	def cartisian_to_polar(self,pt):
		x = pt[0]
		y = pt[1]
		radius = 0
		theta = 0
		radius = math.sqrt((x*x)+(y*y))
		if radius == 0:
			radius = 0.00000000001
		if(x == 0):
			if y == 0:
				theta = 0
			elif y > 0:
				theta = math.pi/2
			else:
				theta = math.pi * 3/2.0
		else:
			theta = math.atan(y*1.0/x)
			if(x < 0):
				theta += math.pi

		return radius, theta

	#polar_to_cartisian- convert to cartisian coordinates
	#Can be cleaned up. A bit rough
	def polar_to_cartisian(self,pt):
		radius = pt[0]
		theta = pt[1]
		x = radius * math.cos(theta)
		y = radius * math.sin(theta)
		return x, y


	#shift_to_origin - make the center of the image (0,0)
	def shift_to_origin(self,pt,width,height):
		return ((pt[0] - width/2.0),(-1*pt[1] + height/2.0))


	#shift_to_image - make the center of the image (imgwidth/2,imgheight/2)
	def shift_to_image(self,pt,width,height):
		return ((pt[0] + width/2),(-1*pt[1] + height/2.0))



	#position_to_pixel- project a 3D point onto a 2D image based on camera orientation
	#Units: pixels and radians
	#Can be cleaned up and converted to vector based math
	def position_to_pixel(self, x, y, z, roll, pitch, yaw, vfov,hfov, width, height):

		#tanslation distortion constant
		#high = little distortion
		tk = 3.0
		
		#rotation distortion constant
		#High = little distortion
		rk = 1.0

		#intiailize variables
		alpha = 0
		heading = 0
		imgSize = math.sqrt(math.pow(width,2)+ math.pow(height,2))
		FOV = math.sqrt(math.pow(vfov,2)+ math.pow(hfov,2))

		# not directly under camera
		if x != 0 or y != 0:	
			#convert to polar coordinates
			radius , heading = self.cartisian_to_polar((x,y))

			#rotate image with camera heading
			heading += math.radians(yaw)

			#print "radius %f, heading %f" % (heading, radius)

			#find FOV at point heading
			FOV = (vfov * math.fabs(y)*1.0/radius) + (hfov * math.fabs(x)*1.0/radius)
			#print "fov %f" % FOV
			#find image size at point heading
			imgSize = (height * math.fabs(y)*1.0/radius) + (width * math.fabs(x)*1.0/radius)
			#print "imgsize %f" % imgSize

			alpha = math.atan(radius * 1.0/(z*tk))

		#print "alpha %f" % alpha

		#shift image with camera tilt
		alphaX ,alphaY = self.polar_to_cartisian((alpha,heading))
		betaX = alphaX - roll
		betaY = alphaY - pitch
		sx,sy =  betaX * imgSize* 1.0/ math.radians(FOV/tk),betaY * imgSize* 1.0/ math.radians(FOV/tk)

		#return sx,sy


		#simulate perspective
		if(x < 0):
			if(y > 0):
				#px = sx - (roll * (width/math.pi)) - (pitch * (height/math.pi))
				#py = sy + (roll * (width/math.pi)) + (pitch * (height/math.pi)) 

				px = sx + (pitch * (sy/(math.pi * rk)))
				py = sy - (roll * (sx/(math.pi * rk)))
			else:
				#px = sx - (roll * (width/math.pi)) + (pitch * (height/math.pi))
				#py = sy - (roll * (width/math.pi)) - (pitch * (height/math.pi))
				px = sx + (pitch * (sy/(math.pi * rk)))
				py = sy + (roll * (sx/(math.pi * rk)))
		else:
			if(y > 0):
				#px = sx - (roll * (width/math.pi)) + (pitch * (height/math.pi))
				#py = sy - (roll * (width/math.pi)) + (pitch * (height/math.pi))
				px = sx - (pitch * (sy/(math.pi * rk)))
				py = sy - (roll * (sx/(math.pi * rk)))
			else:
				#px = sx - (roll * (width/math.pi)) - (pitch * (height/math.pi))
				#py = sy + (roll * (width/math.pi)) - (pitch * (height/math.pi))
				px = sx - (pitch * (sy/(math.pi * rk)))
				py = sy + (roll * (sx/(math.pi * rk)))

		return px , py

	#simulate_target - simulate an image given the target position(pixels) in reference to camera and camera orientation(radians)
	def simulate_target(self,dX,dY,dZ,roll,pitch,yaw):
			corners = np.float32([[0,0],[self.target_width ,0],[0,self.target_height],[self.target_width, self.target_width]])
			newCorners = np.float32([[0,0],[0,0],[0,0],[0,0]])


			#calculate projection for four corners of image
			for i in range(0,len(corners)):
				x = corners[i][0]
				y = corners[i][1]

				#position shift in reference to target
				x += dX
				y += dY
				#shift image
				x,y = self.shift_to_origin((x,y),imgHeight,imgWidth)
				#calculate perspective and position
				x , y = self.position_to_pixel(x,y,dZ,roll,pitch,yaw,self.camera_vfov,self.camera_hfov,self.camera_height,self.camera_width) 
				#unshift image
				x,y = self.shift_to_image((x,y),imgHeight,imgWidth)

				newCorners[i] = x,y
				
			#project image
			M = cv2.getPerspectiveTransform(corners,newCorners)
			sim = cv2.warpPerspective(img,M,(imgWidth,imgHeight),borderValue=self.backgroundColor)

			return sim



if __name__ == "__main__":
	pl_sim.PrecisionLandSimulator()
	pl_sim.main()



	



