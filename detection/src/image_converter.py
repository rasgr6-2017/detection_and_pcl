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


# # # Blob detection of different colours # # #


class blobDetect:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)
    self.pixel_pub = rospy.Publisher("pixel_index", Int32MultiArray, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
    self.countdown = 50;
    self.alter = True;

    # self.temp = cv2.imread('/home/ras26/catkin_ws/src/read_image/src/QR.PNG')

  def callback(self,data):
    
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
      
    
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    r,h,c,w = 250,90,400,125  # simply hardcoded the values
    track_window = (c,r,w,h)

# set up the ROI for tracking
    roi = cv_image[r:r+h, c:c+w]
    
    hsv_roi =  cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    (rows,cols,channels) = cv_image.shape
    
    mask = cv2.inRange(hsv_roi, np.array((0., 60.,32.)), np.array((180.,255.,255.)))

    roi_hist = cv2.calcHist([hsv_roi],[0],mask,[180],[0,180])

    cv2.normalize(roi_hist,roi_hist,0,255,cv2.NORM_MINMAX)    
    
    cv2.imshow("hsv", cv_image)
    cv2.waitKey(3)
    
    term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )
    while(1):
	    ret = True
	    if ret == True:
		    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		    dst = cv2.calcBackProject([hsv],[0],roi_hist,[0,180],1)
		# apply meanshift to get the new location
		    ret, track_window = cv2.CamShift(dst, track_window, term_crit)
		# Draw it on image
		    pts = cv2.circle(ret, (20,100), 5, (0,255,0))
		    pts = np.int0(pts)
		    img2 = cv2.polylines(cv_image,[pts],True, 255,2)
		    cv2.imshow('img2',img2)
		    k = cv2.waitKey(60) & 0xff
		    if k == 27:
			    break
		    else:
			    cv2.imwrite(chr(k)+".jpg",img2)
	    else:
		    break
    params = cv2.SimpleBlobDetector_Params()
    params.filterByConvexity = False
    params.filterByInertia = False
    params.filterByArea = True
    params.minArea = 900
    params.maxArea = 40000
	
    detector = cv2.SimpleBlobDetector(params)
    # the hue changes within different camera, and might change with different white balance setting
    bounds = [([0, 0, 80], [10, 255, 255]), #red
    ([80, 150, 0], [115, 255, 255]), #blue
    ([17, 200, 100], [28, 255, 255]), #yellow
    ([8, 180, 100], [12, 255, 255]), #orange
    ([38, 100, 100], [65, 255, 255]), #green
    ([150, 0, 0], [170, 255, 255])] #purple
    	
    for (lower, upper) in bounds:
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")
    	
        mask = cv2.inRange(hsv_image, lower, upper)
        output = cv2.bitwise_and(hsv_image, hsv_image, mask = mask)
        #cv2.imshow("output", output)
        output = cv2.cvtColor(output, cv2.COLOR_HSV2BGR)
        output = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        
        
        ret3,output = cv2.threshold(output,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        
        
        kernel1 = np.ones((10,10),np.uint8)
        output = cv2.erode(output, kernel1,iterations = 1)
        kernel2 = np.ones((10,10),np.uint8)
        kernel2 = np.ones((20,20),np.uint8)
        output = cv2.erode(output, kernel2,iterations = 3)
        
        output = cv2.bitwise_not(output)
        
        keypoints = detector.detect(output)
        
        #gray = cv2.Canny(output, 120, 120)
        #th = 70
        #tw = th
        #cv2.imshow("Image", gray)

        
        # pt shows the float pixel position of center
        # size shows the diameter value
        # print(keypoints[0].pt[0], keypoints[0].pt[1], keypoints[0].size)
        
        # print (len(keypoints))
        index = Int32MultiArray();
        index.data = [-1, -1]
        
        espeakPub = rospy.Publisher('/espeak/string', String, queue_size = 10)
        rospy.init_node('image_converter', anonymous=True)
        rate = rospy.Rate(10)
        
        if len(keypoints) > 0:
            # index = int(keypoints[0].pt[0]) + 640*int(keypoints[0].pt[1])
            index.data = [int(keypoints[0].pt[0]), int(keypoints[0].pt[1])]
            # # Publish to the espeak node saying that it sees a coloured object # #
            if np.any(lower == [0, 0, 80]):          
                colour = "red"
            if np.any(lower == [80, 150, 0]):          
                colour = "blue"
            if np.any(lower == [17, 200, 100]):          
                colour = "yellow"
            if np.any(lower == [8, 180, 100]):          
                colour = "orange"
            if np.any(lower == [38, 100, 100]):          
                colour = "green"

            #if np.any(upper == [170, 255, 255]):
            	#if np.any(lower == [150, 0, 0]):          
                 #   colour = "I see a purple object"
                  #  rospy.loginfo(colour) 
                   # espeakPub.publish(colour)
                    #rate.sleep()
            # self.alter = False
            msg = "I see an object that's", colour
            rospy.loginfo(msg) 
            espeakPub.publish(colour)
            rate.sleep()
            print(keypoints[0].pt[0], keypoints[0].pt[1], keypoints[0].size)
            
        
        im_with_keypoints = cv2.drawKeypoints(output, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    #cv2.imshow("Image window", output)
    #cv2.imshow("detection", im_with_keypoints)
    #cv2.waitKey(3)
    
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    gray = cv2.Canny(gray, 120, 120)
    th = 70
    tw = th
    #cv2.imshow("Image", gray)
      
    #template = cv2.resize(self.temp, (tw, th), interpolation=cv2.INTER_CUBIC)
    #template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    template = cv2.Canny(gray, 120, 120)
    
    result = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF)
    (_, maxValue, minLoc, maxLoc) = cv2.minMaxLoc(result)
    topLeft = maxLoc
    botRight = (topLeft[0] + int(tw*1), topLeft[1] + int(th*1))
    roi = gray[topLeft[1]:botRight[1], topLeft[0]:botRight[0]]
        
    mask = np.zeros(gray.shape, dtype="uint8")
    gray = cv2.addWeighted(gray, 0.25, mask, 0.75, 0)	
    gray[topLeft[1]:botRight[1], topLeft[0]:botRight[0]] = roi
    
            
    cv2.imshow("Template", template)
    cv2.waitKey(3)
    #cv2.imshow("Template", template)
    
		#cv2.imshow("111", cv_image) 

         
#cv2.imshow("Image window", output)
#cv2.imshow("detection", im_with_keypoints)
#cv2.waitKey(3)

    
    

    
    try:
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        self.pixel_pub.publish(index)
        print(index.data) 
    except CvBridgeError as e:
        print(e)

 # # # Canny edge detector for booby trap # # #       

class cannyEdge:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)
        self.pixel_pub = rospy.Publisher("pixel_index", Int32MultiArray, queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
        self.countdown = 50;
        self.alter = True;
        #self.temp = cv2.imread('/home/ras26/catkin_ws/src/read_image/src/QR.PNG')

    def callback(self,data):
    
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
'''
        (rows,cols,channels) = cv_image.shape
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.Canny(gray, 120, 120)
        th = 70
        tw = th
        cv2.imshow("Image", gray)
      
        template = cv2.resize(self.temp, (tw, th), interpolation=cv2.INTER_CUBIC)
        template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        template = cv2.Canny(template, 120, 120)
    
        result = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF)
        (_, maxValue, minLoc, maxLoc) = cv2.minMaxLoc(result)
        topLeft = maxLoc
        botRight = (topLeft[0] + int(tw*1), topLeft[1] + int(th*1))
        roi = gray[topLeft[1]:botRight[1], topLeft[0]:botRight[0]]
        
        mask = np.zeros(gray.shape, dtype="uint8")
        gray = cv2.addWeighted(gray, 0.25, mask, 0.75, 0)	
        gray[topLeft[1]:botRight[1], topLeft[0]:botRight[0]] = roi
'''        
        
#        cv2.imshow("Template", template)
#        cv2.waitKey(3)
 #       cv2.imshow("Template", template)
    
		#cv2.imshow("111", cv_image)    

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  blobDetect()
  #cannyEdge()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
