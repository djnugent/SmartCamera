import cv2
import numpy as np
from matplotlib import pyplot as plt
import math


'''
How to use:
arrrow keys to translate
WASD for roll and pitch
RT for yaw
QE for zoom
'''



def cartisian_to_polar(pt):
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

def polar_to_cartisian(pt):
	radius = pt[0]
	theta = pt[1]
	x = radius * math.cos(theta)
	y = radius * math.sin(theta)
	return x, y



def shift_to_origin(pt,width,height):
	return ((pt[0] - width/2.0),(-1*pt[1] + height/2.0))

def shift_to_image(pt,width,height):
	return ((pt[0] + width/2),(-1*pt[1] + height/2.0))




def position_to_pixel(x,y, alt, roll, pitch, yaw, vfov,hfov, width, height):

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
		radius , heading = cartisian_to_polar((x,y))

		#print "radius %f, heading %f" % (heading, radius)
		heading += yaw

		#find FOV at point heading
		FOV = (vfov * math.fabs(y)*1.0/radius) + (hfov * math.fabs(x)*1.0/radius)
		#print "fov %f" % FOV
		#find image size at point heading
		imgSize = (height * math.fabs(y)*1.0/radius) + (width * math.fabs(x)*1.0/radius)
		#print "imgsize %f" % imgSize

		alpha = math.atan(radius * 1.0/(alt*tk))

	#print "alpha %f" % alpha

	#shift image with camera tilt
	alphaX ,alphaY = polar_to_cartisian((alpha,heading))
	betaX = alphaX - roll
	betaY = alphaY - pitch
	sx,sy =  betaX * imgSize* 1.0/ math.radians(FOV/tk),betaY * imgSize* 1.0/ math.radians(FOV/tk)

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


def simulate_target(img,dX,dY,dZ,roll,pitch,yaw):
		imgHeight, imgWidth, chan =  img.shape
		corners = np.float32([[0,0],[imgWidth ,0],[0,imgHeight],[imgWidth , imgHeight]])
		newCorners = np.float32([[0,0],[0,0],[0,0],[0,0]])



		for i in range(0,len(corners)):
			x = corners[i][0]
			y = corners[i][1]

			#position shift in reference to target
			x +=dX
			y += dY
			#shift image
			x,y = shift_to_origin((x,y),imgHeight,imgWidth)
			#calculate perspective and position
			x , y = position_to_pixel(x,y,dZ,roll,pitch,yaw,60,60,imgHeight,imgWidth) 
			#unshift image
			x,y = shift_to_image((x,y),imgHeight,imgWidth)

			newCorners[i] = x,y
			
		#project image
		M = cv2.getPerspectiveTransform(corners,newCorners)

		sim = cv2.warpPerspective(img,M,(imgWidth,imgHeight),borderValue=(234,152,289))

		return sim












targetX = 0
targetY = 0
targetZ = 0
cameraX = 0
cameraY = 0
cameraZ = 300
cameraRoll = 0
cameraPitch = 0
cameraYaw = 0


img = cv2.imread('drawing.png')



while True:

	key = cv2.waitKey(0)
	print key

	#forward
	if(key ==1113938):
		cameraY += 100
	#backward
	if(key ==1113940):
		cameraY -= 100
	#left
	if(key ==1113937):
		cameraX -= 100
	#right
	if(key ==1113939):
		cameraX += 100
	#pitch(+)
	if(key == 1048695):
		cameraPitch += 0.05
	#pitch(-)
	if(key == 1048691):
		cameraPitch -= 0.05
	#roll(+)
	if(key == 1048676):
		cameraRoll += 0.05
	#roll(-)
	if(key == 1048673):
		cameraRoll -= 0.05
	#yaw(+)
	if(key == 1048690):
		cameraYaw += 0.05
	#yaw(-)
	if(key == 1048692):
		cameraYaw -= 0.05
	#zoomIn
	if(key == 1048677):
		cameraZ -= 100
	#zoomOut
	if(key == 1048689):
		cameraZ += 100

	dX = targetX - cameraX
	dY = targetY - cameraY
	dZ = cameraZ - targetZ

	#print dX,dY,dZ,cameraRoll,cameraPitch



	sim = simulate_target(img,dX,dY,dZ,cameraRoll,cameraPitch,cameraYaw)



	cv2.imshow('Sim', sim)


	