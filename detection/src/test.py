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


#### Blob detection of different colours and Canny edge detector ####


class detect:

  # Get camera info

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)
    self.pixel_pub = rospy.Publisher("pixel_index", Int32MultiArray, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.blob)
    self.countdown = 50;
    self.alter = True;

    self.temp = cv2.imread('/home/ras26/catkin_ws/src/read_image/src/QR.PNG')

  def blob(self,data):
    
    self.countdown = self.countdown-1;
    # print(self.countdown)
    if self.countdown != 49:
        #print(self.countdown)
        if self.countdown == 0:
            self.countdown = 50
        return
    
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
      
    
    (rows,cols,channels) = cv_image.shape

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
       
    params = cv2.SimpleBlobDetector_Params()

    # detect using different params

    params.filterByConvexity = False

    ############## TEST #################
    params.filterByCircularity = True
    params.minCircularity = 0.5
    #####################################
    
    params.filterByInertia = False

    params.filterByArea = True
    params.minArea = 900
    params.maxArea = 10000
	
    detector = cv2.SimpleBlobDetector(params)

    # hsv values for each colour we want to detect, hsv changes with different camera
    bounds = [([0, 0, 80], [10, 255, 255]), #red
    ([80, 150, 0], [115, 255, 255]), #blue
    ([17, 200, 100], [28, 255, 255]), #yellow
    ([8, 180, 100], [12, 255, 255]), #orange
    ([38, 100, 100], [65, 255, 255]), #green
    ([150, 0, 0], [170, 255, 255])] #purple
    	
    for (lower, upper) in bounds:
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")

    	# make a hsv mask of video stream with upper and lower limits
        mask = cv2.inRange(hsv, lower, upper)
        output = cv2.bitwise_and(hsv, hsv, mask = mask)
        # convert to BGR
        output = cv2.cvtColor(output, cv2.COLOR_HSV2BGR)
        # convert to grey 
        grey = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        
        # uses binary and Otsu's thresholding
        ret3, thresholdedIm = cv2.threshold(grey,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        '''
        kernel1 = np.ones((10,10),np.uint8)
        output = cv2.erode(output, kernel1,iterations = 1)
        kernel2 = np.ones((10,10),np.uint8)
        kernel2 = np.ones((20,20),np.uint8)
        output = cv2.erode(output, kernel2,iterations = 3)
		'''

		################# TEST ###################

		# using closing (dilation followed by erosion) to fill gaps
		kernel = np.ones((10,10),np.uint8)
        closing = cv2.morphologyEx(thresholdedIm, cv2.MORPH_CLOSE, kernel)
        
        output = cv2.bitwise_not(closing)
        
        keypoints = detector.detect(output)
  
        # find contours
        im2, contours, hierarchy = cv2.findContours(thresholdedIm,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv2.imshow("Contours", contours)
        cv2.waitKey(3) 
        
        # draw contours onto vid stream
        contourIm = cv2.drawContours(cv_image, contours, -1, (0,255,0), 3)
        cv2.imshow("Contoured image", contourIm)
        cv2.waitKey(3)        
        ##########################################

        index = Int32MultiArray();
        index.data = [-1, -1]

        #### Publish to the espeak node saying that it sees whatever coloured object ####

        espeakPub = rospy.Publisher('/espeak/string', String, queue_size = 10)
        rospy.init_node('image_converter', anonymous=True)
        rate = rospy.Rate(10)
        
        if len(keypoints) > 0:
            # index = int(keypoints[0].pt[0]) + 640*int(keypoints[0].pt[1])
            index.data = [int(keypoints[0].pt[0]), int(keypoints[0].pt[1])]
           
            if np.any(lower == [0, 0, 80]):          
                colour = "I see a red object"
            if np.any(lower == [80, 150, 0]):          
                colour = "I see a blue object"
            if np.any(lower == [17, 200, 100]):          
                colour = "I see a yellow object"
            if np.any(lower == [8, 180, 100]):          
                colour = "I see an orange object"
            if np.any(lower == [38, 100, 100]):          
                colour = "I see a green object"
            if np.any(lower == [150, 0, 0]):
            	colour = "I see a purple object"

            rospy.loginfo(colour) 
            espeakPub.publish(colour)
            rate.sleep()
            print(keypoints[0].pt[0], keypoints[0].pt[1], keypoints[0].size)
            
        
        im_with_keypoints = cv2.drawKeypoints(output, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    cv2.imshow("Image window", output)
    cv2.imshow("detection", im_with_keypoints)
    cv2.waitKey(3)
    
    ################## TEST ###################

    # edge detector

    edges = cv2.Canny(grey, 120, 120)
    th = 70
    tw = th
    cv2.imshow("Edges", edges)
      
    #template = cv2.resize(self.temp, (tw, th), interpolation=cv2.INTER_CUBIC)
    #template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)

    # match QR code to obstacle
    ## maybe we can use this for the shape detection ## 
    template = cv2.Canny(temp, 120, 120)
    
    result = cv2.matchTemplate(edges, template, cv2.TM_CCOEFF)
    (_, maxValue, minLoc, maxLoc) = cv2.minMaxLoc(result)
    topLeft = maxLoc
    botRight = (topLeft[0] + int(tw*1), topLeft[1] + int(th*1))
    roi = gray[topLeft[1]:botRight[1], topLeft[0]:botRight[0]]
        
    mask = np.zeros(edges.shape, dtype="uint8")
    grey = cv2.addWeighted(edges, 0.25, mask, 0.75, 0)	
    #grey[topLeft[1]:botRight[1], topLeft[0]:botRight[0]] = roi
    
    cv2.imshow("Template", template)        
    cv2.imshow("Result", result)
    cv2.waitKey(3)
    ###########################################


    try:
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        self.pixel_pub.publish(index)
        print(index.data) 
    except CvBridgeError as e:
        print(e)
  

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  detect()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

