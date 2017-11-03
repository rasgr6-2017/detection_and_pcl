#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('simple_detection')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import datetime
import sys, termios, tty, os, time
PATH=os.path.dirname(os.path.abspath(__file__))

    
#### Blob detection of different colours and Canny edge detector ####
class colourClass:
    keypoint=None
    output=None
    mask=None
    im_with_keypoints=None
    edges=None
    
class detect:

    redBounds = [([0,100,70], [10,255,255])]
    blueBounds = [([80,150,0], [115,255,255])]
    yellowBounds = [([17, 100, 100], [25, 255, 255])]
    orangeBounds = [([8, 0, 100], [16, 255, 255])]
    greenBounds = [([30, 100, 53],  [80, 255, 255])]
    purpleBounds = [([110, 0, 0], [150, 255, 255])]
    hsv=None
    
    # save to this dictonary [output, keypoint]
    redClass=colourClass()
    blueClass=colourClass()
    yellowClass=colourClass()
    orangeClass=colourClass()
    purpleClass=colourClass()
    greenClass=colourClass()
    
    outputs={'red':redClass,   'blue': blueClass,  'yellow':yellowClass,  'orange': orangeClass,  'green': greenClass,  'purple': purpleClass }

   # templates dict with list of shapes 
    templates={'cube':[],  'hollow_cube':[],  'sphere':[],  'star':[],  'cross':[],  'triangle':[] }

    
  # Get camera info-

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)
        self.pixel_pub = rospy.Publisher("pixel_index", Int32MultiArray, queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.blob)
        self.countdown = 50;
        self.alter = True;
        self.initTemplateImages() # pre-process template images for shape detection

    #self.temp = cv2.imread('/home/ras26/catkin_ws/src/read_image/scripts/QR.PNG')

    def blob(self,data):
        
        self.countdown = self.countdown-1;
        if self.countdown != 9:
        #print(self.countdown)
            if self.countdown == 0:
                self.countdown = 10
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
          

        (rows, cols, channels) = cv_image.shape
        self.hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        cv2.imshow("hsv", cv_image)
        cv2.waitKey(3)

            
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
        
            
        self.detector = cv2.SimpleBlobDetector(params)
        
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
        for col in self.outputs:

            if not (self.outputs[col].keypoint ==None): # if color detected
                if (totalMask == None): 
                    totalMask = self.outputs[col].mask # first color detected
                else: 
                    totalMask = cv2.bitwise_and(totalMask,  self.outputs[col].mask) # add masks together from all colors that where found
                
                if  (totalOutput == None): 
                    totalOutput = self.outputs[col].output
                else:
                    totalOutput = cv2.bitwise_and(totalOutput, self.outputs[col].output) # add outputs together
                
                if (total_im_with_keypoints==None): 
                    total_im_with_keypoints = self.outputs[col].im_with_keypoints 
                else: 
                    total_im_with_keypoints=cv2.bitwise_and(total_im_with_keypoints, self. outputs[col].im_with_keypoints) # add keypoints together
                detected+= col + ", "
                
                
        if not (totalMask==None): # something found
            print (detected)
            cv2.imshow("Total Detection", total_im_with_keypoints) # show all keypoints together
            cv2.waitKey(3)
            
        # now we should find the shapes and then classify what object it is.
        # then choose wich object that is closest or has more points?
        # say a sentance in the speaker and send command to arm
        #self.hough(cv_image)

            

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

            index = Int32MultiArray();
            index.data = [-1, -1]           
            
            # get keypoint
            keypoints = self.detector.detect(output)
            #save keypoint to dictionay
            self.outputs[col].keypoint=keypoints
            
            colour = ""
            if len(keypoints)  >  0:
                
                # get shape on binary image
                edges = cv2.Canny(grey,  10,  200) # get edges, threshold1 and 2
                contours, hierarchy = cv2.findContours(edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(edges, contours, -1, (255,0,0), 2)
                cv2.imshow('Edge Detection', edges)
                cv2.waitKey(3)
                self.outputs[col].edges = edges # save its shape to dict
                
                
                
                index.data = [int(keypoints[0].pt[0]), int(keypoints[0].pt[1])]
                colour = "I see a " + col + " object"
                #print (colour)
                if (col=='red'): circleColour=(0, 0, 255)
                elif (col=='green'): circleColour=(0, 255, 0)
                elif (col=='blue'): circleColour=(255, 0, 0)
                elif (col=='yellow'): circleColour=(9, 240, 250)
                elif (col=='purple'): circleColour=(185, 9, 250)
                elif (col=='orange'): circleColour=(9, 97, 250)
                # draw circle at keypoint for matched colour
                im_with_keypoints = cv2.drawKeypoints(output, keypoints, np.array([]), circleColour, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                self.outputs[col].im_with_keypoints=im_with_keypoints # save to dictionary
                self.shapeDetector( edges)
                
                
            else:
                # reset values if nothing found
               self.outputs[col].output = None
               self.outputs[col].mask = None
               self.outputs[col].keypoint = None
               self.outputs[col].im_with_keypoints = None
               self.outputs[col].edges=None
        # clear variables
        
        return  
      

#### Publish to the espeak node saying that it sees whatever coloured object #### 
    def speak(self, colour):
      
        espeakPub = rospy.Publisher('/espeak/string', String, queue_size = 10)
        rospy.init_node('image_converter', anonymous=True)
        rate = rospy.Rate(0)
    
        rospy.loginfo(colour) 
        espeakPub.publish(colour)
        
#### Shape detector ####
    def shapeDetector(self,  shape):
        # take pictures of all objects and name them after shape 
        # current shape = cannyEdgeDetection(output)
        # exclude some shapes after color
        # matchShapes() with preprocessed templates (or use SIFT?)
        # send pixel coord if found to PCL node
        #templateShape=self.templates['triangle'][0]
        #ret = cv2.matchShapes(templateShape,  shape,  cv2.cv.CV_CONTOURS_MATCH_I1,  0)
        #print(ret)
        return 
        
    def initTemplateImages(self):
        # preprocess local images 
        #convert to binary shapes just like the camera image
        print("Pre-processing template images...")
        
        for shapeType in self.templates:
            directory=PATH+'/objects/'+shapeType+'/'
            for filename in os.listdir(directory): # go through all template images in the folder
                if filename.endswith(".png"): # is a png
                    img = cv2.imread(directory+filename) # read image from local disk
                    # ->>>>>need to mask for its color like in bounds function!!!!!!!!!!!!!!!! -<<<<
                    # otherwise we see other edges in background
                    grey=self.imgToBinary(img) # process image
                    edges = cv2.Canny(grey,  50,  50*3) # get edges, threshold1 and 2
                    self.templates[shapeType].append(edges) # save shape data to list
                    #cv2.imshow("Shape", edges)
                    #cv2.waitKey(3)
        
        print("Done!")
        return
    
    def imgToBinary(self,  img):
        grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # to binary
        ret, thresholdedIm = cv2.threshold(grey,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        return grey
        
    # write image to file in local folder
    def takePicture(self,  image):
        print(PATH)
        cv2.imwrite(PATH+datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")+".png",image)
        return
        
    def hough(self,  bgr): # this works mroe for 2D, maybe for booby trap ?
    
        #bgr = cv2.cvtColor(self.hsv, cv2.COLOR_HSV2BGR)
        # convert to grey 
        grey = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
       
        edges = cv2.Canny(grey, 50, 50*3, apertureSize = 3)
        
        lines = cv2.HoughLines(edges,1,np.pi/180,200)
        
        
        if not (lines == None): # lines detected
            #print(lines)
            for rho,theta in lines[0]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
                    
                cv2.line(bgr,(x1,y1),(x2,y2),(0,0,255),2)
             
            
            cv2.imshow("Shape Detector", bgr)
            cv2.waitKey(3)
      
    #template = cv2.resize(self.temp, (tw, th), interpolation=cv2.INTER_CUBIC)
    #template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)

    # match QR code to obstacle
    ## maybe we can use this for the shape detection ## 
    #template = cv2.Canny(temp, 120, 120)
    
    #result = cv2.matchTemplate(edges, template, cv2.TM_CCOEFF)
    #(_, maxValue, minLoc, maxLoc) = cv2.minMaxLoc(result)
    #topLeft = maxLoc
    #botRight = (topLeft[0] + int(tw*1), topLeft[1] + int(th*1))
    #roi = gray[topLeft[1]:botRight[1], topLeft[0]:botRight[0]]
        
    #mask = np.zeros(edges.shape, dtype="uint8")
    #grey = cv2.addWeighted(edges, 0.25, mask, 0.75, 0)	
    #roi = grey[topLeft[1]:botRight[1], topLeft[0]:botRight[0]]
    
    #cv2.imshow("Template", template)        
    #cv2.imshow("Result", grey)
    #cv2.waitKey(3)
    ###########################################


#try:
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
 #   self.pixel_pub.publish(index)
  #  print(index.data) 
#except CvBridgeError as e:
 #   print(e)
 
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


