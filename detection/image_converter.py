#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('simple_detection')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Time
from std_msgs.msg import UInt8
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import datetime
import sys, termios, tty, os, time
PATH=os.path.dirname(os.path.abspath(__file__)) # path of this file
SHOW_IMAGE=True # for debugging

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
    
class detect:

    redBounds = [([0,100,150], [13,255,255])]
    blueBounds = [([80,50,200], [115,150,255])]
    yellowBounds = [([17, 100, 50], [30, 255, 255])]
    orangeBounds = [([6, 150, 120], [16, 255, 255])]
    greenBounds = [([30, 50, 0],  [80, 255, 255])]
    purpleBounds = [([140, 50, 50], [179, 255, 255])]
    hsv=None
    raw_image=None
    
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
    
  # Get camera info-

    def __init__(self):
        self.image_pub = rospy.Publisher("image_output",Image, queue_size=10)
        self.pixel_pub = rospy.Publisher("pixel_index", Int32MultiArray, queue_size=10)
        self.espeakPub = rospy.Publisher('/espeak/string', String, queue_size = 10)
        self.eviPubStr = rospy.Publisher('/ras_msgs/RAS_Evidence', String, queue_size = 10)
        self.eviPubIm = rospy.Publisher('/ras_msgs/RAS_Evidence', Image, queue_size = 10)
        self.eviPubInt = rospy.Publisher('/ras_msgs/RAS_Evidence', UInt8, queue_size = 10)
        self.eviPubTime = rospy.Publisher('/ras_msgs/RAS_Evidence', Time, queue_size = 10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.blob)
        self.countdown = 50;
        self.alter = True;
        self.initTemplateImages() # pre-process template images for shape detection
       

    def blob(self,data):
        
        self.countdown = self.countdown-1;
        if self.countdown != 9:
        #print(self.countdown)
            if self.countdown == 0:
                self.countdown = 50
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
          

        (rows, cols, channels) = cv_image.shape
        self.hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        self.raw_image=cv_image
        #cv2.imshow("hsv", cv_image)
        #cv2.waitKey(3)

            
        params = cv2.SimpleBlobDetector_Params()
    
    # detect using different params

        params.filterByConvexity = False
        params.filterByCircularity = False
        params.filterByInertia = False

        params.filterByArea = True
        params.minArea = 900
        params.maxArea = 40000
        
        # save as png image for templates (comment out this)
        #keypress  = self.getch()
        #if (keypress=='t'):
            #print("saving picture...")
            #self.takePicture(cv_image)
        #cv2.imshow("RAW image", self.raw_image) # show match
        #cv2.waitKey(3)  
        
        self.detector = cv2.SimpleBlobDetector(params)
        
        ### Battery detection ###
        self.checkForBattery()
        # booby trap QR code detection
        self.checkForBoobyTrap()
            
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
        detected = "I found : "
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
                
                if not self.outputs[col].shapeName==None: # could detect shape also
                    detectedShapeColor +=col + " " + self.outputs[col].shapeName + ", "
                detected+= col + " and "
                
                
        if not (totalMask==None): # some color found
            #print (detected)
            if not (detectedShapeColor=="I see a "):
                print(detectedShapeColor)
                pixel_coordinate = self.chooseObject() # choose object and get pixel coordinate

                try: # send pixel to pcl node
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.raw_image, "bgr8"))
                    hello = self.pixel_pub.publish(pixel_coordinate)                  
                    self.speak(detectedShapeColor) # send string to speaker
                    print(hello)

                except CvBridgeError as e:
                    print(e)
                
            if SHOW_IMAGE:
                cv2.imshow("Color Detection", total_im_with_keypoints) # show all keypoints together
                cv2.waitKey(3)

    # choose object from list of objects
    def chooseObject(self):
        for object in self.outputs:
            if not self.outputs[object].shapeName==None: # go through all found objects
                # take the one with highest point or random
                return self.outputs[object].index # pixel coord
       
       
    def checkForBoobyTrap(self):
		 ### QR reader  booby trap ###
        
        self.temp = cv2.imread(PATH+'/objects/obstacle/QR.png', 0)
        grey = cv2.cvtColor(self.raw_image, cv2.COLOR_BGR2GRAY)  
        edges = cv2.Canny(grey, 120, 120)
         
        template = np.resize(self.temp, (70,70,3))
        template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        template = cv2.Canny(template, 120, 120)
        
        result = cv2.matchTemplate(edges, template, cv2.TM_CCOEFF)
        (_, maxValue, minLoc, maxLoc) = cv2.minMaxLoc(result)
        topLeft = maxLoc
        botRight = (topLeft[0] + int(70*1), topLeft[1] + int(70*1))
        roi = grey[topLeft[1]:botRight[1], topLeft[0]:botRight[0]]
        mask = np.zeros(edges.shape, dtype="uint8")
        grey = cv2.addWeighted(edges, 0.25, mask, 0.75, 0)
        
        grey[topLeft[1]:botRight[1], topLeft[0]:botRight[0]] = roi
        if maxValue > 2000000:
            detected = "I see an obstacle"
            #pixelCoord = self.chooseObject() # wrong, this is only for shapes...
            print(detected)
            #print(pixelCoord)
            self.speak(detected)
            txt = open(PATH+"/mapFiles/obstacle.txt", "w")

            txt.write("obstacle" + '\n')
            txt.close()
                
    def checkForBattery(self):
		image = cv2.cvtColor(self.raw_image, cv2.COLOR_BGR2GRAY)
		#laplacian = cv2.Laplacian(image,cv2.CV_32F)
		#cv2.imshow("Laplacian", laplacian)
		
		
		
		for model in self.batteryTemplates: 
			#model = cv2.Laplacian(model,cv2.CV_32F)

			w, h = model.shape[::-1]
########################## Trying ORB ###############################			
			#res = cv2.matchTemplate(image, model, cv2.TM_CCOEFF_NORMED)
			#(_, maxValue, minLoc, maxLoc) = cv2.minMaxLoc(res)
			#mask = np.zeros(image.shape, dtype="uint8")
			
			#cv2.imshow("Lap", model)
			#loc = np.where( res >= threshold)
			match=False
			orb = cv2.ORB()
			kp = orb.detect(model, None)
			kp, des = orb.compute(model, kp)
			kp1 = orb.detect(image, None)			
			kp1, des1 = orb.compute(image, kp1)
			bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True)
			clusters = np.array([des])
			bf.add(clusters)
			
			matches = bf.match(des, des1)
			
			
			#thres = (sum(dist)/len(dist))*0.5

			image2 = cv2.drawKeypoints(image, kp1, color=(0,255,0), flags = 0)
			cv2.imshow("Battery Detection", image2)
			threshold = 1100 # max distance 
			if len(matches) > 100: # minimum number of matches
				# Sort them in the order of their distance.
				matches = sorted(matches, key = lambda x:x.distance)

				if matches[10].distance < threshold: # filter bad matches 
					print("Found battery.")
					print(kp1[matches[0].queryIdx].pt) # pixel pos
					return True

			#for pt in zip(*loc[::-1]):
			#	print("I've found the battery!!")
			#	match=True
		return match
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
            # using closing (dilation followed by erosion) to fill gaps
            kernel = np.ones((10,10),np.uint8)
            
            output = cv2.erode(thresholdedIm, kernel,iterations = 1)
            kernel2 = np.ones((10,10),np.uint8)
            output = cv2.dilate(output, kernel2,iterations = 1)
            kernel2 = np.ones((20,20),np.uint8)
            output = cv2.erode(output, kernel2,iterations = 3)
            output = cv2.bitwise_not(output)
            #closing = cv2.dilate(output, kernel2, iterations = 1)
            #closing = cv2.morphologyEx(thresholdedIm, cv2.MORPH_CLOSE, kernel)

            index = Int32MultiArray()
            index.data = [-1, -1]           
            
            # get keypoint
            keypoints = self.detector.detect(output)
            #save keypoint to dictionay
            self.outputs[col].keypoint=keypoints
            cv2.imshow('color detection', output)
            cv2.waitKey(3)

            colour = ""
            if len(keypoints)  >  0:
                
                # get shape on binary image
                edges = cv2.Canny(grey,  50,  150) # get edges, threshold1 and 2
                contours, hierarchy = cv2.findContours(edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(edges, contours, -1, (255,0,0), 2)
#                cv2.imshow('Edge Detection', edges)
#                cv2.waitKey(3)
                self.outputs[col].edges = edges # save its shape to dict
                
                index.data = [int(keypoints[0].pt[0]), int(keypoints[0].pt[1])]
                self.outputs[col].index=index
                
                colour = "I see a " + col + " object"
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
        
        ## TEMPLATE MATCHING  almost working ######
        shapeMask = self.raw_image
        shapeMask1 = cv2.cvtColor(shapeMask, cv2.COLOR_BGR2GRAY)
        threshold = 0.77
        rectangle_color=self.strToBGR(color)
        # go through all possible shape templates (except some)
        for shape in self.templates: 
            # exclude some possiblities depending on color   
            if self.checkIfShapeExist(shape,  color):
                for model in self.templates[shape]: # go through all models
                    #image = cv2.Canny(shapeMask1, 50, 200)
                    #model = cv2.Canny(model, 50, 200)
                    #cv2.imshow("image", image)
                    w, h = model.shape[::-1]
                    
                    res = cv2.matchTemplate(image, model, cv2.TM_CCOEFF_NORMED)
                    (_, maxValue, minLoc, maxLoc) = cv2.minMaxLoc(res)
                    topLeft = maxLoc
                    botRight = (topLeft[0] + int(70*1), topLeft[1] + int(70*1))
                    roi = shapeMask1[topLeft[1]:botRight[1], topLeft[0]:botRight[0]]
        
                    mask = np.zeros(image.shape, dtype="uint8")
                    shapeMask1 = cv2.addWeighted(image, 0.25, mask, 0.75, 0)
        
                    shapeMask1[topLeft[1]:botRight[1], topLeft[0]:botRight[0]] = roi
                    
                    loc = np.where( res >= threshold)
                    match=False
                    
                    
                    
                    for pt in zip(*loc[::-1]):
                        cv2.rectangle(shapeMask, pt, (pt[0] + w, pt[1] + h), rectangle_color, 2) # match rectangle
                        #print("It's a "+ color + " " +  shape)
                        self.outputs[color].shapeName=shape # save to dictionary
                        match=True

                        ############################## TEST ##############################
                        # Make .txt file with object_id and it's coordinates in relation to the robot #
                        object_id = color+"_"+shape
                        group_number = 6
                        stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        objectCoord = open(PATH+ "/mapFiles/" + object_id + ".txt", "w")

                        ## only need x, y coordinates - need to get rid of z #
                        

                        objectCoord.write(stamp + '\n')
                        objectCoord.write(stamp + '\n')
                        objectCoord.close()

                        ## publish object_id and group_number to RAS_Evidence.msg #

                        #self.eviPubStr.publish(object_id)
                        #self.eviPubInt.publish(group_number)
                        #self.eviPubTime.publish(stamp)
                        ## publish shapeMask to RAS_Evidence.msg #
                        #image_evidence = shapeMask
                        #self.eviPubIm.publish(image_evidence)

                        #bag = rosbag.Bag(object_id + ".bag", 'w')

                        #try:
                            #bag.write('evidence', stamp + '\n' + group_number + '\n' + image_evidence + '\n' + object_id + '\n' + image_evidence + '\n' + pixel_coordinate)

                        #finally:
                            #bag.close()

                    if SHOW_IMAGE:   
                        cv2.imshow("Object Recogntion", shapeMask) # show match
                        cv2.waitKey(3)
                        ###################################################################
                    if match:
                        return shapeMask# done matching
                    
        # Grey mask of current image should have bigger area
        # take more model images, more angles
        # hollow_cube vs cylinder is very difficult
        # Purple hsv bounds need to be sorted
        # cross is very bad, because of purple mask being too small
        
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
                    edges = cv2.Canny(grey,  50,  150) # get edges, threshold1 and 2
                    contours, hierarchy = cv2.findContours(edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(edges, contours, -1, (255,0,0), 2)
                    self.templates[shapeType].append(grey) # save shape data to list
#                    print(shapeType+" - "+filename)
#                    print(len(contours))
#                    print(" ")
#                    cv2.imshow("Shape2", grey)
#                    cv2.waitKey(3)
#                    self.getch()
        
        d= PATH+'/objects/battery/' # init battery images
        for f in os.listdir(d):
			if f.endswith(".png"):
				img = cv2.imread(d+f, 0) # read image from local disk
				self.batteryTemplates.append(img)
        print("Done!")
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
        
    def hough(self,  edges,  img): # this works mroe for 2D, maybe for booby trap ?
    
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



