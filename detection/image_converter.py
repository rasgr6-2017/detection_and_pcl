#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('simple_detection')
import sys
import rospy
import cv2
import rosbag
import numpy as np
from sys import argv
from std_msgs.msg import String
from std_msgs.msg import Time
from std_msgs.msg import UInt8
from std_msgs.msg import Int32MultiArray, Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Transform
from geometry_msgs.msg import PoseStamped, TransformStamped
from ras_msgs.msg import RAS_Evidence
from cv_bridge import CvBridge, CvBridgeError
import datetime
import time
import zbar
from PIL import Image as pilim
from matplotlib import pyplot as plt
import sys, termios, tty, os, time
PATH=os.path.dirname(os.path.abspath(__file__)) # path of this file
SHOW_IMAGE=True # for debugging
BATTERY_TIMEOUT=8 # ignore battery if found x seconds earlier
#### Blob detection of different colours and Canny edge detector ####
class colourClass:
    keypoint=None
    output=None
    mask=None
    im_with_keypoints=None
    edges=None
    shape=None
    shapeName=None
    index=None
    detected_last_frame=False
    
class detect:

    redBounds = [([0,40,0], [6,255,255])]
    blueBounds = [([100,100,0], [109,255,210])]
    yellowBounds = [([15, 100, 80], [29, 255, 180])]
    orangeBounds = [([5, 150, 50], [12, 255, 255])]
    greenBounds = [([60,50,0], [90,255,200])]
    purpleBounds = [([110, 40, 0], [135, 255, 255])]
    hsv=None
    raw_image=None
    raw_data=None
    battery_detected_before=False
    time_since_battery_found=BATTERY_TIMEOUT
    
    # save to this dictonary [output, keypoint]
    redClass=colourClass()
    blueClass=colourClass()
    yellowClass=colourClass()
    orangeClass=colourClass()
    purpleClass=colourClass()
    greenClass=colourClass()
    
    outputs={'red':redClass,   'blue': blueClass,  'yellow':yellowClass,  'orange': orangeClass,  'green': greenClass,  'purple': purpleClass }

   # templates dict with list of shapes 
    templates={'cube':[],  'hollow_cube':[],  'sphere':[],  'star':[], 'cylinder':[], 'cross':[],  'triangle':[] }
    
    batteryTemplates=[]
    xPos=0 # robot pose
    yPos=0
    zPos=0
    orientation=0
    
  # Get camera info-

    def __init__(self):
		# Publishers
        self.image_pub = rospy.Publisher("image_output",Image, queue_size=10)
        self.pixel_pub = rospy.Publisher("pixel_index", Int32MultiArray, queue_size=10)
        self.espeakPub = rospy.Publisher('/espeak/string', String, queue_size = 10)
        self.eviPub = rospy.Publisher('/evidence', RAS_Evidence, queue_size = 10)
		# Subscribers
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.blob)
        self.coordinates = rospy.Subscriber("target_coord", Point, self.callback)
        self.barcodeSub = rospy.Subscriber("/barcode", String, self.boobyCallback)
        self.robotPos = rospy.Subscriber("/localization/pose", PoseStamped, self.robotPosCallback)
        
        self.countdown = 50;
        self.alter = True;
        self.initTemplateImages() # pre-process template images for shape detection
        self.x = 0
        self.y = 0

        
        self.barcode1 = String()
		
    def blob(self,data):
        
        self.countdown = self.countdown-1;
        if self.countdown != 9:
        #print(self.countdown)
            if self.countdown == 0:
                self.countdown = 15
            return
            
        try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			self.data = data
        except CvBridgeError as e:
            print(e)
          

        (rows, cols, channels) = cv_image.shape
        
        cv_image = cv_image[100: 500, 0: 1000]
        self.hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        self.raw_image=cv_image
        #cv2.imshow(self.hsv)
       # hist = cv2.calcHist([self.hsv], [0, 1], None, [180, 256], [0, 180, 0, 256])
        #plt.imshow(hist,interpolation = 'nearest')
        #plt.show()

            
        params = cv2.SimpleBlobDetector_Params()
    
    # detect using different params

        params.filterByConvexity = False
        params.filterByCircularity = False
        params.filterByInertia = False

        params.filterByArea = True # 
        params.minArea = 10 # min size of detection
        params.maxArea = 70000
        # Change thresholds
        params.minThreshold = 1
        
        # save as png image for templates (comment out this)
        #keypress  = self.getch()
        #if (keypress=='t'):
            #print("saving picture...")
            #self.takePicture(cv_image)
        cv2.imshow("RAW image", self.raw_image) # show match
        cv2.waitKey(3)  
        
        self.detector = cv2.SimpleBlobDetector(params)
        
        ### Battery detection ###
        self.checkForBattery()
        # booby trap QR code detection
        QRcode = cv2.imread(PATH+'/objects/obstacle/QR.png', 0)
        self.checkForBoobyTrap(QRcode)
            
        ##################
            
        # check all colours
        self.bound(self.blueBounds,  "blue")
        self.bound(self.redBounds,  "red")
        self.bound(self.greenBounds,  "green")
        self.bound(self.purpleBounds,  "purple")
        self.bound(self.yellowBounds,  "yellow")
        self.bound(self.orangeBounds,  "orange")
        
         #add masks together, add outputs together, show images, say/print what we saw in this iteration
        
        totalMask=None
        total_im_with_keypoints=None
        totalOutput=None
        detectedShapeColor="I see a "
        for col in self.outputs:
            if not (self.outputs[col].keypoint == None): # if color detected
                if (totalMask == None): 
                    totalMask = self.outputs[col].mask # first color detected
                else: 
                    totalMask = cv2.bitwise_and(totalMask,  self.outputs[col].mask) # add masks together from all colors that where found

                if (total_im_with_keypoints==None): 
                    total_im_with_keypoints = self.outputs[col].im_with_keypoints 
                else: 
                    total_im_with_keypoints=cv2.bitwise_and(total_im_with_keypoints, self. outputs[col].im_with_keypoints) # add keypoints together
                
                if (not self.outputs[col].shapeName==None) and (self.outputs[col].detected_last_frame): # save to dictionary: # could detect shape twice also
                    detectedShapeColor +=col + " " + self.outputs[col].shapeName + ", "
                    
                
                
        if not (totalMask==None): # some color found
            if not (detectedShapeColor=="I see a "):
                print(detectedShapeColor)
                pixel_coordinate = self.chooseObject() # choose object and get pixel coordinate

                try: # send pixel to pcl node
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.raw_image, "bgr8"))
                    self.pixel_pub.publish(pixel_coordinate)                 
                    self.speak(detectedShapeColor) # send string to speaker

                except CvBridgeError as e:
                    print(e)
                
            if SHOW_IMAGE:
                cv2.imshow("Color Detection", total_im_with_keypoints) # show all keypoints together
                cv2.waitKey(1)

    # choose object from list of objects
    def chooseObject(self):
        for object in self.outputs:
            if not self.outputs[object].shapeName==None: # go through all found objects
                # take the one with highest point or random
                return self.outputs[object].index # pixel coord
       
       
    def checkForBoobyTrap(self, image):

		## Reader
		scanner = zbar.ImageScanner()
		## Configure it
		scanner.parse_config('enable')
		

		grey = cv2.cvtColor(self.raw_image, cv2.COLOR_BGR2GRAY)
		pil = pilim.fromarray(grey)
		width, height = pil.size
		raw = pil.tobytes()
		## Wrap image data
		zbar_im = zbar.Image(width, height, 'Y800', raw)
		## Scan image
		scanner.scan(zbar_im)
		#print(n)
		for symbol in zbar_im: # found booby trap
			print(symbol.type, symbol.data)
			self.speak("It's a trap!")		
			 
		return
	
	

    def checkForBattery(self):
		image = cv2.cvtColor(self.raw_image, cv2.COLOR_BGR2GRAY)
		threshold = 0.77
		pixCoord = []
		time1=0
		for model in self.batteryTemplates: 
			w, h = model.shape[::-1]
			
			res = cv2.matchTemplate(image, model, cv2.TM_CCOEFF_NORMED)
			(_, maxValue, minLoc, maxLoc) = cv2.minMaxLoc(res)

			loc = np.where( res >= threshold)


			for pt in zip(*loc[::-1]):
				
				### Pixel coordinates of battery ###
				
				pixCoord.append(maxLoc)
				a = np.asarray(pixCoord)
				time1 = time.time() - self.time_since_battery_found
			## Make sure it sees the battery at least twice and long time since last battery
				if self.battery_detected_before and time1>BATTERY_TIMEOUT: #seconds since found
					self.time_since_battery_found=time.time() # get current time 
					self.speak("I've found the battery!!") 
					self.battery_detected_before=False
					print("battery found")
					## Pixel coordinates are the top left of the matched image
					# x coordinate + (width of matched template / 2)
					xBattery = maxLoc[0] + (w/2)
					# y coordinates + (height of matched template / 2)
					yBattery = maxLoc[1] + (h/2)
					
					# Write coordinates to .txt file
					X = self.xPos # + ? some math based on robots orientation and pixel coord 
					Y = self.yPos
					batteryCoord = open(PATH+ "/mapFiles/battery.txt", "w")
					batteryCoord.write(str(X) + ' ' + str(Y)+'/n') # for mapping
					batteryCoord.close()
					return
					
				else:
					self.battery_detected_before=True # will detect next frame
					return

		self.battery_detected_before=False # never detected
		return

		
########################################################################            
    
    # make hsv masks of video stream with upper and lower limits
    def bound(self, boundaries,  col):

        for (lower, upper) in boundaries:
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")
            
            mask = cv2.inRange(self.hsv, lower, upper)
            # save output to dictionary
            self.outputs[col].mask=mask
            
            output = cv2.bitwise_and(self.hsv, self.hsv, mask = mask)
            
            # convert to BGR
            output = cv2.cvtColor(output, cv2.COLOR_HSV2BGR)
            # convert to grey         

            grey = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
    
            # uses binary and Otsu's thresholding
            ret, thresholdedIm = cv2.threshold(grey,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

            index = Int32MultiArray()
            index.data = [-1, -1]           
            
            # get keypoint
            keypoints = self.detector.detect(output)
            #save keypoint to dictionay
            self.outputs[col].keypoint=keypoints
            #cv2.imshow("color test", grey) # show match
            #cv2.waitKey(3)

            if len(keypoints)  >  0:
                index.data = [int(keypoints[0].pt[0]), int(keypoints[0].pt[1])]
                print(col)
                self.outputs[col].index=index
                
                circleColour=self.strToBGR(col)
                # draw circle at keypoint for matched colour
                im_with_keypoints = cv2.drawKeypoints(output, keypoints, np.array([]), circleColour, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                self.outputs[col].im_with_keypoints=im_with_keypoints # save to dictionary
                shape=self.shapeDetector( grey,  col )
                self.outputs[col].shape=shape
                
                
            else:
                # reset values if nothing found
               self.outputs[col].output = None
               self.outputs[col].mask = None
               self.outputs[col].keypoint = None
               self.outputs[col].im_with_keypoints = None
               self.outputs[col].edges=None
               self.outputs[col].shape=None
               self.outputs[col].shapeName=None
               self.outputs[col].index=None
               self.outputs[col].detected_last_frame=False
        # clear variables
        
        return  
      

    def strToBGR(self,  col):
        if (col=='red'): return (0, 0, 255)
        elif (col=='green'): return (0, 255, 0)
        elif (col=='blue'): return (255, 0, 0)
        elif (col=='yellow'): return (9, 240, 250)
        elif (col=='purple'): return (185, 9, 250)
        elif (col=='orange'): return (9, 97, 250)

#### Publish to the espeak node saying that it sees whatever coloured object #### 
    def speak(self,  str):
          
        #rospy.loginfo(self.speakString) 
        self.espeakPub.publish(str)
        
#### Shape detector ####
    def shapeDetector(self,  image,  color):
        
        shapeMask = self.raw_image
        shapeMask1 = cv2.cvtColor(shapeMask, cv2.COLOR_BGR2GRAY)
        threshold = 0.77
        rectangle_color=self.strToBGR(color)
        # go through all possible shape templates (except some)
        counter=0
        for shape in self.templates: 
            # exclude some possiblities depending on color   
            if self.checkIfShapeExist(shape,  color):
                for model in self.templates[shape]: # go through all models

                    w, h = model.shape[::-1]
                    
                    res = cv2.matchTemplate(image, model, cv2.TM_CCOEFF_NORMED)
                    (_, maxValue, minLoc, maxLoc) = cv2.minMaxLoc(res)
                    topLeft = maxLoc

                    loc = np.where( res >= threshold)
                    match=False
                    
                    
                    
                    for pt in zip(*loc[::-1]):
                        cv2.rectangle(shapeMask, pt, (pt[0] + w, pt[1] + h), rectangle_color, 2) # match rectangle

                        self.outputs[color].shapeName = shape # save to dictionary
                        self.outputs[color].detected_last_frame = True # save to dictionary
                        counter += 1 # check how many matches
                        match = True
                        
                        if counter > 70: # filter out false big matches
							self.outputs[color].shapeName = None
							self.outputs[color].detected_last_frame = False # save to dictionary
							match=False
							break
                        
                        
                    if match: 
						print(counter)
						cv2.imshow("matched model", model) # show match
						cv2.waitKey(3)
                        # Make .txt file with object_id and it's coordinates in relation to the robot #
						object_id = color+"_"+shape

						stamp = rospy.get_rostime()
						## x, y coordinates on MAP
						X = self.xPos # + ? some math based on robots orientation + fake distance
						Y = self.yPos # + ? 
						
						objectCoord = open(PATH+ "/mapFiles/" + object_id + ".txt", "w")
						objectCoord.write(object_id + ' ' + str(X) + ' ' + str(Y) +'/n') # position of object on map
						objectCoord.close()

						pos=TransformStamped()
						pos.header = Header()
						pos.transform=Transform()
						pos.transform.translation.x=X
						pos.transform.translation.y=Y
						pos.header.stamp=stamp
						## publish object_id and group_number to RAS_Evidence.msg #
					
						msg_data=RAS_Evidence()
						msg_data.stamp=stamp
						msg_data.group_number = 6
						msg_data.image_evidence=self.data
						header = Header()
						msg_data.image_evidence.header=header
						msg_data.object_id=object_id
						msg_data.object_location=pos
						

						self.eviPub.publish(msg_data)


						#bag = rosbag.Bag(object_id + ".bag", 'w')

						#try:
						#    bag.write('evidence', str(self.x))

						#finally:
						#    bag.close()

						if SHOW_IMAGE:   
							cv2.imshow("Object Recogntion", shapeMask) # show match
							cv2.waitKey(3)
                        ###################################################################
                    if match:
                        return shapeMask # done matching
                    else:
						self.outputs[color].detected_last_frame=False # save to dictionary
                    
        # Grey mask of current image should have bigger area
        # take more model images, more angles
        # hollow_cube vs cylinder is very difficult

        
        return None
        
    
    def checkIfShapeExist(self,  shape,  color):
        #print(shape)
        if shape=='cube':
            if color=='green' or color=='blue' or color=='yellow':
               return True
        elif shape=='hollow_cube':
            if color=='red' or color=='green':
                return True
        elif shape=='sphere':
            if color=='red' or color=='yellow':
                return True
        elif shape=='triangle':
            if color=='blue':
                return True
        elif shape=='cylinder':
            if color=='red' or color=='green':
                return True
        elif shape=='cross':
            if color=='purple' or color=='orange':
                return True
        elif shape=='star':
            if color=='orange' or color=='purple':
                return True
        return False
        
    def initTemplateImages(self):
        # preprocess local images 
        #convert to binary shapes just like the camera image
        print("Pre-processing template images...")
        
        for shapeType in self.templates:
            directory=PATH+'/objects/'+shapeType+'/'
            for filename in os.listdir(directory): # go through all template images in the folder
                if filename.endswith(".png"): # is a png
                    img = cv2.imread(directory+filename) # read image from local disk
                    img = self.maskColor(filename, img) # mask color to remove background
                    grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # to binary
                    ret, thresholdedIm = cv2.threshold(grey,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
                    self.templates[shapeType].append(grey) # save shape data to list
                    
                    #cv2.imshow("Shape2", grey)
                    #cv2.waitKey(3)
                    #print(shapeType+" - "+filename)
                    #self.getch()
        # This is only getting the first image #
        d= PATH+'/objects/battery/' # init battery images
        for f in os.listdir(d):
			if f.endswith(".png"):
				img = cv2.imread(d+f,0) # read image from local disk
				self.batteryTemplates.append(img)
        print("Done!")
        #print(self.batteryTemplates)
        return
        
    def maskColor(self,  file, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # to hsv
        output=None
        # check what color the image model is
        if (file.startswith('red')):
            boundaries=self.redBounds
        elif (file.startswith('blue')):
            boundaries=self.blueBounds
        elif (file.startswith('yellow')):
            boundaries=self.yellowBounds
        elif (file.startswith('green')):
            boundaries=self.greenBounds
        elif (file.startswith('orange')):
            boundaries=self.orangeBounds
        elif (file.startswith('purple')):
            boundaries=self.purpleBounds
            
        for (lower, upper) in boundaries:
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")
            
            mask = cv2.inRange(hsv, lower, upper)
            output = cv2.bitwise_and(hsv, hsv, mask = mask)
            
            # convert to BGR
            output = cv2.cvtColor(output, cv2.COLOR_HSV2BGR)
        return output

    # write image to file in local folder
    def takePicture(self,  image):
        print(PATH)
        cv2.imwrite(PATH+datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")+".png",image)
        return
        
    def hough(self,  edges,  img): # this works more for 2D, maybe for booby trap ?
    
        #bgr = cv2.cvtColor(self.hsv, cv2.COLOR_HSV2BGR)
        # convert to grey 
        #grey = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        lines = cv2.HoughLinesP(edges, rho = 1, theta = 1*np.pi/180, threshold = 50, minLineLength = 70, maxLineGap = 20)
        
        if not (lines == None): # lines detected
            
            if len(lines[0]) < 100: # filter errors
                i=0
                for leftx, boty, rightx, topy in lines[0]:
                    cv2.line(img,(leftx, boty), (rightx,topy),(0,0,255),2)
                    i += 1
                   # if (i > 3):
                        #break
                if SHOW_IMAGE:
                    cv2.imshow("Shape Detector", img)
                    cv2.waitKey(3)
    
    def callback(self, msg):
		#self.x = msg.x
		#self.y = msg.y		
		return
		  		
    # get keyboard press
    def getch(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
                            
    def boobyCallback(self, data):
		self.barcode = data
		return self.barcode
		
			
			
    def robotPosCallback(self, data):
		self.xPos = data.pose.x
		self.yPos = data.pose.y
		self.zPos = data.pose.z
		self.orientation = data.rotation
		#print(self.xPos)
		return self.xPos, self.yPos, self.zPos	
		

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  detection = detect()
    
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)



