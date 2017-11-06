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
PATH=os.path.dirname(os.path.abspath(__file__)) # path of this file

    
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

    redBounds = [([0,100,70], [10,255,255])]
    blueBounds = [([80,150,0], [115,255,255])]
    yellowBounds = [([17, 100, 100], [25, 255, 255])]
    orangeBounds = [([8, 0, 100], [16, 255, 255])]
    greenBounds = [([30, 100, 53],  [80, 255, 255])]
    purpleBounds = [([110, 0, 0], [150, 255, 255])]
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
            
        self.detector = cv2.SimpleBlobDetector(params)
        
        #cv2.imshow("RAW image", self.raw_image) # show match
        #cv2.waitKey(3)
            
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
            if not (self.outputs[col].keypoint ==None): # if color detected
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
                detected+= col + ", "
                
                
        if not (totalMask==None): # some color found
            #print (detected)
            if not (detectedShapeColor=="I see a "):
                print(detectedShapeColor)
                pixel_coordinate = Int32MultiArray()
                pixel_coordinate.data=[-1, -1]  
                pixel_coordinate.data=self.chooseObject() # choose object and get pixel coordinate
                self.speakString=detectedShapeColor # speak this

                try:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.raw_image, "bgr8"))
                    self.pixel_pub.publish(pixel_coordinate)
                    #print(pixel_coordinate) 
                except CvBridgeError as e:
                    print(e)
                
            cv2.imshow("Color Detection", total_im_with_keypoints) # show all keypoints together
            cv2.waitKey(3)
            
                
        # now choose which one is best! (biggest/credits)
        # send to pcl
            
            
    # choose object from list of objects
    def chooseObject(self):
        for object in self.outputs:
            if not self.outputs[object].shapeName==None: # go through all found objects
                # take the one with highest point or random
                return object.index # pixel coord
                
        
        
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
                self.outputs[col].index=index.data
                
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
    def speak(self):
      
        espeakPub = rospy.Publisher('/espeak/string', String, queue_size = 10)
        rospy.init_node('image_converter', anonymous=True)
        rate = rospy.Rate(1)
    
        #rospy.loginfo(self.speakString) 
        espeakPub.publish(self.speakString)
        
#### Shape detector ####
    def shapeDetector(self,  image,  color):
        # take pictures of all objects and name them after shape 
        # current shape = cannyEdgeDetection(output)
        # exclude some shapes after color
        # matchShapes() with preprocessed templates (or use SIFT?)
        # send pixel coord if found to PCL node
        #templateShape = self.templates['triangle'][0]
        #print(ret)
        #templateContour = cv2.imread(PATH+"/objects/hollow_cube/red_0.png", 0) 
       
        
        #w, h = templateContour.shape[::-1]
        
        #contours = cv2.cvtColor(self.raw_image, cv2.COLOR_BGR2GRAY)
        # ignore if bounding area too big?
        # fit lines?
#        sortedContours = sorted(contours, key=cv2.contourArea,  reverse=True)  # sort after biggest area
#        for c in sortedContours :
#            for tempC in templateContour:
#                ret = cv2.matchShapes(tempC,  c,  cv2.cv.CV_CONTOURS_MATCH_I1,  0)
#                if (ret <= 0.1):
#                    #print(ret)
#                    cv2.drawContours(img3, c, -1, (255,0,0), 2)
#                    cv2.imshow("match", img3)
#                    cv2.waitKey(3)
#               
                
        #self.hough(contours,  img3)
        
        #templateContour = self.templates['hollow_cube'][0] # test image
        
        ## TEMPLATE MATCHING  almsot working ######
        shapeMask=self.raw_image
        threshold = 0.82
        rectangle_color=self.strToBGR(color)
        # go through all possible shape templates (except some)
        for shape in self.templates: 
            # exclude some possiblities depending on color   
            if self.checkIfShapeExist(shape,  color):
                for model in self.templates[shape]: # go through all models
                    w, h = model.shape[::-1]
                    res = cv2.matchTemplate(image, model, cv2.TM_CCOEFF_NORMED)
                    loc = np.where( res >= threshold)
                    match=False
                    for pt in zip(*loc[::-1]):
                        cv2.rectangle(shapeMask, pt, (pt[0] + w, pt[1] + h), rectangle_color, 2) # match rectangle
                        #print("It's a "+ color + " " +  shape)
                        self.outputs[color].shapeName=shape # save to dictionary
                        match=True
                    cv2.imshow("Object Recogntion", shapeMask) # show match
                    cv2.waitKey(3)
                    if match:
                        return shapeMask# done matching
                    
        # Grey mask of current image should have bigger area
        # ignore very large color detections (floor, wall, etc)
        # take more model images, more angles
        # hollow_cube vs Cube is very difficult
        # Star has not been tested...
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
                    img=self.maskColor(filename,  img) # mask color to remove background
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
                cv2.imshow("Shape Detector", img)
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
                            
                    
    def drawMatches(self,  img1, kp1, img2, kp2, matches):
#    
#    My own implementation of cv2.drawMatches as OpenCV 2.4.9
#    does not have this function available but it's supported in OpenCV 3.0.0

        # Create a new output image that concatenates the two images together
        # (a.k.a) a montage
        rows1 = img1.shape[0]
        cols1 = img1.shape[1]
        rows2 = img2.shape[0]
        cols2 = img2.shape[1]

        out = np.zeros((max([rows1,rows2]),cols1+cols2,3), dtype='uint8')

        # Place the first image to the left
        out[:rows1, :cols1] = np.dstack([img1])

        # Place the next image to the right of it
        out[:rows2, cols1:] = np.dstack([img2])

        # For each pair of points we have between both images
        # draw circles, then connect a line between them
        for mat in matches:

            # Get the matching keypoints for each of the images
            img1_idx = mat.queryIdx
            img2_idx = mat.trainIdx

            # x - columns
            # y - rows
            (x1,y1) = kp1[img1_idx].pt
            (x2,y2) = kp2[img2_idx].pt

            # Draw a small circle at both co-ordinates
            # radius 4
            # colour blue
            # thickness = 1
            cv2.circle(out, (int(x1),int(y1)), 4, (255, 0, 0), 1)
            cv2.circle(out, (int(x2)+cols1,int(y2)), 4, (255, 0, 0), 1)

            # Draw a line in between the two points
            # thickness = 1
            # colour blue
            cv2.line(out, (int(x1),int(y1)), (int(x2)+cols1,int(y2)), (255, 0, 0), 1)

        # Show the image
        # cv2.imshow('Matched Features', out)
        # cv2.waitKey(0)
        # cv2.destroyWindow('Matched Features')

        # Also return the image if you'd like a copy
        return out

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


