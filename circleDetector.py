
import cv2
import numpy as np
import math
import time



current_milli_time = lambda: int(round(time.time() * 1000))


'''
TODO:
make ECCENTRICITY dependent on craftAttitude

'''


class circleDetector(object):

	ECCENTRICITY = 0.6 #how round a circle needs to be. Perfect circle = 1
	DISTANCETHRESHOLD = 15 #acceptable distance(pixels) between cocentric circle centers
	MINCIRCLES = 5 # number of circles needed for a valid target(times 2) 2 circles are often overlayed
	RADIUSTOLERANCE = 2 #pixels: used to identify repeat circles(stacked circles). Problem caused by findContours()
	RATIOTOLERANCE = 0.015 #Tolerance used in comparing actaul ratios and preceived ratios 

	#target specific data
	ringData = np.array([0.8,0.91,0.76,0.84,0.7,0.66,0.49])
	outerRingSize = 0.08255 #radius of outer ring in meters

	#camera info
	cam_hfov = 70.42
	cam_vfov = 43.3

	def __init__(self):
		pass

	'''
	detectTarget
	public method which takes in an image and aircraft orientation and locates the target
	'''
	def analyze_frame(self, child_conn, img, craftAttitude):

		self.finalTarget = None

		#start timer
		start = current_milli_time()


		#blur image and grayscale
		#img = cv2.medianBlur(img,5)
		cimg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)


		#canny edge detector
		edges = cv2.Canny(cimg,100,200,3)

		if edges is not None:


			#locate contours
			contours, hierarchy = cv2.findContours(edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

			#turn contours into ellipses
			circles = np.empty((len(contours)),object)
			circlesCnt = 0
			for i in range(0,len(contours)):
				contour = contours[i]
				#make sure contour contains enough point for an ellipse
				if(len(contour) > 4):
					#detect an ellipse
					ellipse = cv2.fitEllipse(contour)
					#only take ellipses which are round
					if self.checkEccentricity(ellipse,self.ECCENTRICITY):
						circles[circlesCnt] = ellipse
						circlesCnt += 1


			#if circles were found then we look for nested circles
			if circlesCnt > 0:
				#get rid of null elements
				circles = np.resize(circles,circlesCnt)
				#look for nested ellipses
				nestedCircles = self.detectNested(circles)

				#if at least MINCIRCLES circles are nested look for target
				#Times MINCRICLES by two because we haven't removed repeat/stacked circles yet
				if len(nestedCircles) > (self.MINCIRCLES * 2):

					self.finalTarget, center = self.findCommonCenter(nestedCircles)
					if self.finalTarget is not None:
						#we found the objects position on xy-plane


						ratios = self.tagAspectRatio(self.finalTarget)


						#try to calculate distance to target
						if ratios is not None:
							distance = self.calcDistToTarget(self.finalTarget,ratios)

							stop = current_milli_time()
							child_conn.send((stop-start,center, distance, self.finalTarget))
							return

						else: #unable to calculate distance
							stop = current_milli_time()
							child_conn.send(( stop-start, center, -1, self.finalTarget))
							return


			#return runtime, center, distance, targetEllipses
		stop = current_milli_time()
		child_conn.send((stop-start,None,None,None))
		return



	def setParams(self, eccentricity, distanceThres, minCircles, radiusTolerance,ratioTolerance):
		self.ECCENTRICITY = eccentricity
		self.DISTANCETHRESHOLD = distanceThres
		self.MINCIRCLES = minCircles
		self.RADIUSTOLERANCE = radiusTolerance
		self.RATIOTOLERANCE = ratioTolerance

	def distCenters(self,ellipse1,ellipse2):
		#distance between centers
		
		distance = math.sqrt(math.pow((ellipse1[0][0]-ellipse2[0][0]),2) + math.pow((ellipse1[0][1] - ellipse2[0][1]),2))
		return distance


	def detectNested(self,rawCircles):
		size = len(rawCircles)
		nestedCircles = np.empty(size, object)
		nestedCnt = 0
		for i in range(0,size):
			nested = False
			for j in range(i, size):
				if i != j:
					circle1 = rawCircles[i]
					circle2 = rawCircles[j]
					#average major and minor axises
					radius1 = (circle1[1][0] + circle1[1][1]) /2.0
					radius2 = (circle2[1][0] + circle2[1][1]) /2.0

					distance = self.distCenters(circle1,circle2)
						
					#check if a circle is nested within another circle
					if(distance < math.fabs(radius1 - radius2)):
						nested = True
			#add the base circle if it is nested
			if nested:
				nestedCircles[nestedCnt] = rawCircles[i]
				nestedCnt += 1	
		#remove null objects
		nestedCircles  = np.resize(nestedCircles,nestedCnt)

		return nestedCircles



	def checkEccentricity(self,ellipse, threshold):
		#threshold = 1 for perfect circles
		if ellipse[1][0] * 1.0/ ellipse[1][1] > threshold:
			return True
		return False
	'''
	def findCommonCenter(ellipses):

		size = len(ellipses)
		minDistance = 640 #image width
		center = 0,0
		for i in range(0,size):
			nested = False
			for j in range(i, size):
				ellipse1 = ellipses[i]
				ellipse2 = ellipses[j]
				distance = math.sqrt(math.pow((ellipse1[0][0]-ellipse2[0][0]),2) + math.pow((ellipse1[0][1] - ellipse2[0][1]),2))
				if distance <= minDistance and i != j:
					minDistance = distance
					center = ellipse1[0][0], ellipse1[0][1]
		
		return center

	def ellipsesAroundCenter(ellipses, center, threshold):
		size = len(ellipses)
		centeredEllipses = np.empty(size,object)
		centeredCnt = 0
		for i in range(0,size):
				distance = math.sqrt(math.pow((ellipses[i][0][0]-center[0]),2) + math.pow((ellipses[i][0][1] - center[1]),2))
				if distance <= threshold:
					centeredEllipses[centeredCnt] = ellipses[i]
					centeredCnt += 1
		centeredEllipses = np.resize(centeredEllipses,centeredCnt)
		return centeredEllipses
	'''

	def findCommonCenter(self,nestedCircles):

		size = len(nestedCircles)

		#sort by radius
		for i in range(0,size):
			baseCircle = nestedCircles[i]
			smallestRadius = (baseCircle[1][0] + baseCircle[1][1]) /2.0
			smallest = i

			for j in range(i,size):
				circle = nestedCircles[j]
				radius = (circle[1][0] + circle[1][1]) /2.0
				if(radius < smallestRadius):
					smallestRadius = radius
					smallest = j

			nestedCircles[i] = nestedCircles[smallest]
			nestedCircles[smallest] = baseCircle

		#look at all circles
		#add all circles that are within a certain threshold distance
		#compare circle pairs and see which one has the most circles
		concentricCombos = np.empty([size,size],object)


		#start with the largest circle and scan all smaller circles and see if it is concentric with the large circle
		maxConcentricCnt = 1
		maxConcentricIndex = 0

		#stores circle centers
		xSum = np.zeros(size)
		ySum = np.zeros(size)

		for i in range(size-1,0,-1):
			outer = nestedCircles[i]
			concentricCombos[i][0] = outer
			cnt = 1
			

			for j in range(i, 0, -1):
				inner = nestedCircles[j]
				#outer circle and inner circle have same center, are different
				if (self.distCenters(outer,inner) < self.DISTANCETHRESHOLD) and (i != j):
					#check that the circle isn't a repeat(a problem with findContours)
					previous = concentricCombos[i][cnt -1]
					radPrev = (previous[1][0] + previous[1][1]) /2.0
					radCurr = (inner[1][0] + inner[1][1]) /2.0
					#if the circle is cocentric and unique, add it
					if(radPrev - radCurr) > self.RADIUSTOLERANCE:
						concentricCombos[i][cnt] = inner

						xSum[i] += inner[0][0]
						ySum[i] += inner[0][1]

						cnt += 1

			if(cnt > maxConcentricCnt):
				maxConcentricCnt = cnt
				maxConcentricIndex = i

		#no concentric circles
		if(maxConcentricCnt < self.MINCIRCLES):
			return None,None

		#choose the circle set with the most concentric circles
		mostConcentric = concentricCombos[maxConcentricIndex]
		mostConcentric = np.resize(mostConcentric, maxConcentricCnt)

		#calculate meanCenter
		meanCenter = xSum[maxConcentricIndex] / maxConcentricCnt, ySum[maxConcentricIndex]/maxConcentricCnt

		return mostConcentric, meanCenter

	def tagAspectRatio(self,target):
		size = len(target)
		#ratios = np.empty((size-1)*size/2.0, float)
		ratios = np.empty(size-1,float)
		cnt = 0

		for i in range(0,size-1):
			circle1 = target[i]
			circle2 = target[i+1]
			radius1 = (circle1[1][0] + circle1[1][1]) /2.0
			radius2 = (circle2[1][0] + circle2[1][1]) /2.0

			
			ratio = radius2 / radius1
			ratios[cnt] = round(ratio,3)
			cnt += 1
		return ratios			



	# pixels_to_angle_x - converts a number of pixels into an angle in radians 
	def pixels_to_angle_x(self, num_pixels):
		return num_pixels * math.radians(self.cam_hfov) / 640

	# get_distance_from_pixels - returns distance to balloon in meters given number of pixels in image and expected 0.5m radius
	#    size_in_pizels : diameter or radius of the object on the image (in pixels)
	#    actual_size : diameter or radius of the object in meters
	def get_distance_from_pixels(self,size_in_pixels, actual_size):
		 # avoid divide by zero by returning 9999.9 meters for zero sized object 
	    if (size_in_pixels == 0):
	        return 9999.9
	    # convert num_pixels to angular size
	    return actual_size / self.pixels_to_angle_x(size_in_pixels)


	def calculateRingSize(self,ringNumber):
		radius = self.outerRingSize #in meters

		#actualRadius Outer ring size * ratio[n] * ratios[n + 1] ...
		for i in range(0,ringNumber):
			radius = radius * self.ringData[i]

		return radius #in meters



	def calcDistToTarget(self,target, ratios):
		distance = 0
		readings = 0
		for i in range(0,len(ratios)):
			ratio = ratios[i]
			for j in range(0,len(self.ringData)):

				
				if(math.fabs(self.ringData[j] - ratio) <= self.RATIOTOLERANCE):
					circle1 = target[i] #outer ring
					circle2 = target[i+1] #inner ring
					radius1 = (circle1[1][0] + circle1[1][1]) /2.0
					radius2 = (circle2[1][0] + circle2[1][1]) /2.0

					dist1 = self.get_distance_from_pixels(radius1, self.calculateRingSize(j))
					dist2 = self.get_distance_from_pixels(radius2, self.calculateRingSize(j+1))
					distance += (dist1 + dist2 )/2.0
					
					
					readings += 1
		
		#can not decode target	
		if(readings == 0):
			return -1
		#average all distance readings
		return distance/(readings * 1.0)

	# place a marker on image where the target is suppossed to be based on GPS
	def add_predicted_location(self,img,location):
		pass

	#place a bubble on image to indicate a line perpendicular to level
	def add_artifical_horizon(self,img):
		pass

	#put a colored border around the imgae to signify the control the computer has
	def add_mode_border(self,img):
		pass

	#highlight hhe detected target, 1.the target outline 2.each ring
	def add_target_highlights(self, img, target):
		if target is not None:
			for i in range(0,len(target)):
				cv2.ellipse(img,target[i],(0,255,0),2)
		return img