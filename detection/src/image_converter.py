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

class image_converter:

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

    (rows,cols,channels) = cv_image.shape
    
    #gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    #gray = cv2.Canny(gray, 120, 120)
    
    #th = 70
    #tw = th
    
    #template = cv2.resize(self.temp, (tw, th), interpolation=cv2.INTER_CUBIC)
    #template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    #template = cv2.Canny(template, 120, 120)
    
    
    #result = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF)
    #(_, maxValue, minLoc, maxLoc) = cv2.minMaxLoc(result)
    #topLeft = maxLoc
    #botRight = (topLeft[0] + int(tw*1), topLeft[1] + int(th*1))
    #roi = gray[topLeft[1]:botRight[1], topLeft[0]:botRight[0]]
    
    #mask = np.zeros(gray.shape, dtype="uint8")
    #gray = cv2.addWeighted(gray, 0.25, mask, 0.75, 0)	
    #gray[topLeft[1]:botRight[1], topLeft[0]:botRight[0]] = roi
    
    #cv2.imshow("Image", gray)
    # cv2.imshow("Template", template)
    #cv2.waitKey(3)
    # cv2.imshow("Template", template)
    
    # cv2.imshow("111", cv_image)
    
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    params = cv2.SimpleBlobDetector_Params()
    params.filterByConvexity = False
    params.filterByInertia = False
    params.filterByArea = True
    params.minArea = 900
    params.maxArea = 40000
	
    detector = cv2.SimpleBlobDetector(params)
    # the hue changes within different camera, and might change with different white balance setting
    redBound = [([0, 0, 80], [10, 255, 255]),]
    blueBound = [([80, 150, 0], [115, 255, 255])]
    yellowBound = [([17, 200, 100], [28, 255, 255])]
    orangeBound = [([8, 180, 100], [12, 255, 255])]
    greenBound = [([38, 100, 100], [65, 255, 255])]
    pinkBound = [([150, 0, 0], [170, 255, 255])]
	
    for (lower, upper) in blueBound:
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")
    	
    	mask = cv2.inRange(hsv_image, lower, upper)
    	output = cv2.bitwise_and(hsv_image, hsv_image, mask = mask)
        output = cv2.cvtColor(output, cv2.COLOR_HSV2BGR)
        output = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
	
        ret3,output = cv2.threshold(output,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    
        
        kernel1 = np.ones((10,10),np.uint8)
        output = cv2.erode(output, kernel1,iterations = 1)
        kernel2 = np.ones((10,10),np.uint8)
        # output = cv2.dilate(output, kernel2,iterations = 1)
        kernel2 = np.ones((20,20),np.uint8)
        output = cv2.erode(output, kernel2,iterations = 3)
    
        output = cv2.bitwise_not(output)
	    
	
        keypoints = detector.detect(output)
        
        # pt shows the float pixel position of center
        # size shows the diameter value
        # print(keypoints[0].pt[0], keypoints[0].pt[1], keypoints[0].size)
        
        # print (len(keypoints))
        index = Int32MultiArray();
        index.data = [-1, -1]
        if len(keypoints) > 0:
            # index = int(keypoints[0].pt[0]) + 640*int(keypoints[0].pt[1])
            index.data = [int(keypoints[0].pt[0]), int(keypoints[0].pt[1])]
            # self.alter = False
            print(keypoints[0].pt[0], keypoints[0].pt[1], keypoints[0].size)
            
        """
        else:
            index = int(keypoints[1].pt[0]) + 640*int(keypoints[1].pt[1])
            self.alter = True
            print(keypoints[1].pt[0], keypoints[1].pt[1], keypoints[1].size)
        """
        
        im_with_keypoints = cv2.drawKeypoints(output, keypoints, np.array([]), (0, 0, 255),
                                cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
         
    # cv2.imshow("Image window", output)
    cv2.imshow("detection", im_with_keypoints)
    cv2.waitKey(3)
    
    try:
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        self.pixel_pub.publish(index)
        print(index.data) 
    except CvBridgeError as e:
        print(e)
   
    

def main(args):
  rospy.init_node('read_image', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
