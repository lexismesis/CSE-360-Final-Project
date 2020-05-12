#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from movement import MoveRosBots
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
#from std_msgs import String
from geometry_msgs.msg import Point
#from gazebo import common
#from gazebo import color


class ColorDetector(object):

    def __init__(self):
    
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/robot1/camera1/image_raw",Image,self.camera_callback)
        self.moverosbots_object = MoveRosBots()
        self.pub = rospy.Publisher('lanepoint', Point, queue_size=1)

    def camera_callback(self,data):
        
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            
        # We get image dimensions and crop the parts of the image we don't need
        # Bear in mind that because the first value of the image matrix is start and second value is down limit.
        # Select the limits so that it gets the line not too close and not too far, and the minimum portion possible
        # To make process faster.
        height, width, channels = cv_image.shape
        rows_to_watch = 100
        top_trunc = 1*height / 2 #get 3/4 of the height from the top section of the image
        bot_trunc = top_trunc + rows_to_watch #next set of rows to be used
        crop_img = cv_image[top_trunc:bot_trunc, 0:width]
        
        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV).astype(np.float)
        
        # Define the Yellow Colour in HSV
        #RGB
        #[[[222,255,0]]]
        #BGR
        #[[[0,255,222]]]
        """
        To know which color to track in HSV, put in BGR. Use ColorZilla to get the color registered by the camera
        >>> yellow = np.uint8([[[B,G,R ]]])
        >>> hsv_yellow = cv2.cvtColor(yellow,cv2.COLOR_BGR2HSV)
        >>> print( hsv_yellow )
        [[[ 34 255 255]]
        """

        # attempt to get the actual HSV of the gazebo colors used for traffic lights
        #yellow_hsv = gazebo.common.color.yellow.getAsHSV()
        #red_hsv = gazebo.common.color.red.getAsHSV()
        #green_hsv = gazebo.common.color.green.getAsHSV()

        """
        Had to use an HSV color picker to get the upper and lower boundaries
        for each color detection. Provided a smaller boundary so that we only detect 
        the traffic lights, and not other pieces of the scenery.
        """
        # color boundaries for yellow detection
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([50,255,255])

        # color boundaries for red detection
        lower_red = np.array[(359, 40, 90)]
        upper_red = np.array[(359, 100, 100)]

        # color boundaries for green detection
        lower_green = np.array[(100, 60, 100)]
        upper_green = np.array[(113, 95, 100)]

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        # Bitwise-AND yellowmask and original image to get yellow parts
        yellow_mask = cv2.bitwise_and(crop_img,crop_img, mask=mask)

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_red, upper_red)
        # Bitwise-AND yellowmask and original image to get yellow parts
        red_mask = cv2.bitwise_and(crop_img,crop_img, mask=mask)

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_green, upper_green)
        # Bitwise-AND yellowmask and original image to get yellow parts
        green_mask = cv2.bitwise_and(crop_img,crop_img, mask=mask)


        # holder variable for color
        light_color = ""

        # determine which of the 3 colors have been detected and assign Color msg accordingly
        # use the fact that the mask will be 255 if detected, and 0 otherwise.
        # also assign the chosen mask for the camera
        if green_mask[1] > 0:
            light_color = "GREEN"
            chosen_mask = green_mask
        elif red_mask[1] > 0:
            light_color = "RED"
            chosen_mask = red_mask
        elif yellow_mask[1] > 0:
            light_color = "YELLOW"
            chosen_mask = yellow_mask

        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2
        
        
        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        cv2.circle(res,(int(cx), int(cy)), 10,(0,0,255),-1)

        cv2.imshow("Original", cv_image)
        cv2.imshow("HSV", hsv)
        #cv2.imshow("MASK", mask)
        cv2.imshow("RES", chosen_mask)
        
        cv2.waitKey(1)
        
        # Camera coordinates
        p = Point()
        p.x = cx - width / 2
        p.y = cy - height / 2 
        
        
        self.pub.publish(p)

    def clean_up(self):
        self.moverosbots_object.clean_class()
        cv2.destroyAllWindows()
        
        
# make changes to main to adjust for color detection instead of line following
def main():
    rospy.init_node('color_detecting_node', anonymous=True)
    
    
    color_detector_object = ColorDetector()
   
    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        color_detector_object.clean_up()
        rospy.loginfo("shutdown time!")
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        rate.sleep()

    
    
if __name__ == '__main__':
    main()
