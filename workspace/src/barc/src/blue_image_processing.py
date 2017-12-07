#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String, Int32, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from math import sqrt, atan, pi



point_num = 0

# Parameter Default Values
display_image = True
publish_image = True
calibrate_transform = False
image_calibrated = True
calibration_pts = None

global offset_global

offset_global = "not working"

def print_point_callback(event, x, y, flags, param):
    """
    This callback function calibrates the image perspective transform
    """
    global point_num
    global image_calibrated
    global calibration_pts
    global transformation
    global image_processor_global
    
    if event == cv2.EVENT_LBUTTONDOWN:
        point_num = point_num + 1
        if (point_num == 1):
            print('Start at upper left corner. Select points clockwise.')
            calibration_pts = np.float32([[x + 0.0,y + 0.0]])
        elif (point_num <= 4):
            calibration_pts = np.append(calibration_pts, np.float32([[x + 0.0,y + 0.0]]),axis = 0)
        
        if (point_num == 4):
            # Apply Projection Transform
            # ref points [TOPLEFT, TOPRIGHT, BOTTOMRIGHT, BOTTOMLEFT]
            ref_pts = np.float32([[image_processor_global.x_offset,0], \
                [image_processor_global.x_width + image_processor_global.x_offset,0], \
                [image_processor_global.x_width + image_processor_global.x_offset, image_processor_global.y_height], \
                [image_processor_global.x_offset, image_processor_global.y_height]])

            image_processor_global.projection_dim = (image_processor_global.x_width + image_processor_global.x_offset * 2, image_processor_global.y_height)
            image_processor_global.projection_transform = cv2.getPerspectiveTransform(calibration_pts, ref_pts) 
            image_calibrated = True
            cv2.destroyWindow("Calibrate Image Transform")
        display_val ='Pt{}: x:{} y:{}'.format(point_num, x, y)
        print(display_val)

def find_offset_in_lane(img,x,y,width):
    """
    Returns the difference in x and y positions
    operates on pixels. Return value is pixel offset from nominal
    """
    # Starts from the left and right values here
    x_left = 1
    x_right = width-1

    # LEFT calculation, loop from x_left start to width-1 end, looking for the
    # a pixel value >0 and capture as x_left
    while(x_left < width-1 and not img[y, x_left]):
        x_left = x_left + 1

    # RIGHT calculation, loop from x_right start to 1 end, looking for the
    # pixel value >0 and capture as x_right
    while(x_right > 1 and not img[y, x_right]):
        x_right = x_right - 1
    # Returns the tuple
    return (x_left, x_right)



class image_processor:
    """
    This class takes image messages from the USB Camera and converts them to a cv2 format
    subsequently it converts the image to grayscale and performs a perspective and hanning transform.
    Finally, it outputs a delta value indicating the offset of the vehicle from the center of the lane
    """

    def __init__(self):
        global display_image, publish_image, calibrate_transform
        global calibration_pts

        #Create ROS Interfaces
        self.offset_lane_pub = rospy.Publisher("lane_offset", Float32,queue_size=10)
        self.cvfail_pub = rospy.Publisher("cv_abort", Int32, queue_size=10)
       
        self.image_pub = rospy.Publisher("cv_image", Image, queue_size = 10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/barc_cam/image_raw", Image,self.callback_image)
        
        #Get Launch File Parameters and configure node
        calibrate_transform = rospy.get_param("/image_processing/calibrate_transform")
        display_image = rospy.get_param("/image_processing/display_image")
        publish_image = rospy.get_param("/image_processing/publish_image")
        global image_calibrated
        
        # Projection Transform Parameters
        self.x_offset = 100
        self.x_width = 400
        self.y_height = 400
        if calibrate_transform:
            image_calibrated = False
            cv2.namedWindow("Calibrate Image Transform")
            cv2.setMouseCallback("Calibrate Image Transform", print_point_callback)
        else:
            image_calibrated = True
            calibration_pts = np.float32( ([rospy.get_param("/image_processing/upperLeftX"), rospy.get_param("/image_processing/upperLeftY")], \
                                [rospy.get_param("/image_processing/upperRightX"), rospy.get_param("/image_processing/upperRightY")], \
                                [rospy.get_param("/image_processing/lowerRightX"), rospy.get_param("/image_processing/lowerRightY")], \
                                [rospy.get_param("/image_processing/lowerLeftX"), rospy.get_param("/image_processing/lowerLeftY")]))
        
            # Apply Projection Transform
            # ref points [TOPLEFT, TOPRIGHT, BOTTOMRIGHT, BOTTOMLEFT]
            ref_pts = np.float32([[self.x_offset,0], \
                [self.x_width + self.x_offset,0], \
                [self.x_width + self.x_offset, self.y_height], \
                [self.x_offset, self.y_height]])

            self.projection_dim = (self.x_width + self.x_offset * 2, self.y_height)
            self.projection_transform = cv2.getPerspectiveTransform(calibration_pts, ref_pts) 

        self.minimumContourSize = 10
        self.image_threshold = 20
        self.y_offset_pixels_cg = 70
        self.num_nonvalid_img = 0
        self.num_failed_img_before_abort = 30
        self.half_vehicle_width_pixels = (260 // 16) * 6


    def callback_image(self,data):
        """
        Callback for incoming image message
        """
        # Global Variables
        global display_image, publish_image, image_calibrated
        
        # Convert ROS Image Message to CV2 image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape

        #### COMMENTING OUT. CHANGING TO BLUESCALE #########
        # Convert Color Image to Grayscale
        # gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #### END ###########################################
        # Convert Color Image to HSV
        desired_color = np.uint8([[[255,0,0]]])
        hsv_color = cv2.cvtColor(desired_color,cv2.COLOR_BGR2HSV)
        gray_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        # Create a blue color mask
        lower_blue = np.array([hsv_color[0][0][0]-20,150,100])
        upper_blue = np.array([hsv_color[0][0][0]+20,255,255])
        mask = cv2.inRange(gray_image,lower_blue,upper_blue)
        mask = cv2.erode(mask, None, iterations=2)
        # Mask the original image to get pixels only in that range
        res = cv2.bitwise_and(cv_image, cv_image, mask = mask)
        # Re-assign the gray image to our mask
        gray_image = res
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            rect = cv2.minAreaRect(c)
            # rect = ((center(x,y), (width,height), angle of rotation)
            center_x, center_y = rect[0]
            width,height = rect[1]
            rospy.loginfo(str(rect[1]))
            if width > 100:
                
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(gray_image,[box],0,(0,0,255),2)
                center = (int(rect[0][0]),int(rect[0][1]))
                cv2.circle(gray_image,center,3,(0,255,255),-1)
                cv2.putText(gray_image,"center: %s" % str(center),(center[0],center[1]+10),cv2.FONT_HERSHEY_SIMPLEX, .5,(255,255,255))

        cv2.imshow("Blue image", gray_image)
        cv2.waitKey(3)
        


#        ##########################################
#        ####     EDGE DETECTION CODE       #######
#        ##########################################
#
#
#        # Uses Canny edge detection to find high contrast edges in image
#        edges = cv2.Canny(gray_image,75,255)
#        # Initializes lines
#        line_img = np.zeros(cv_image.shape, dtype=np.uint8)
#        # Calls "draw_lines" to find the edges and calculate the offset.
#        # SEE DRAW_LINES
#        offset = draw_lines(line_img,edges)
#        
#        pub = rospy.Publisher('chatter', String, queue_size=10)
#        rate = rospy.Rate(100) # 10hz
#        rospy.loginfo(str(offset))
#        pub.publish(str(offset))
#        rate.sleep()
#        
#        alpha = .6
#        beta = 1.
#        gamma = 0.
#        LinesDrawn2 = cv2.addWeighted(gray_image,alpha,line_img,beta,gamma)
#        
#        cv2.rectangle(LinesDrawn2, (400,100), (640,0), (255,255,255), thickness = -1, lineType = 8, shift = 0) 
#        fontFace = cv2.FONT_HERSHEY_SIMPLEX
#        cv2.putText(LinesDrawn2, "information box", (465, 20), fontFace, .5,(0,0,0))
#        cv2.putText(LinesDrawn2, "steering angle: "+str(offset) + " deg", (440, 50), fontFace, .5,(0,0,0))

#        if display_image or publish_image:
#        ## Publish vehicle steering directions 
#            self.offset_lane_pub.publish(Float32(offset))
#            
#            if display_image:
#            #cv2.imshow("Final", backtorgb)
#                cv2.imshow("Advanced Lane Detection", LinesDrawn2)
#
#
#            if publish_image:
#                try:
#                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(LinesDrawn2, "bgr8"))
#                except CvBridgeError as e:
#                    print(e)
#            else :
#                if display_image:
#                    cv2.imshow("Calibrate Image Transform", edges)
#                if publish_image:
#                    try:
#                        self.image_pub.publish(self.bridge.cv2_to_imgmsg(edges, "bgr8"))
#                    except CvBridgeError as e:
#                        print(e)
#            if display_image:
#                cv2.waitKey(3)

def shutdown_func():
    cv2.destroyAllWindows() 

image_processor_global = None


def draw_lines(img, edges, color=[0, 0,255], thickness=3):


    height, width = edges.shape         
    offset = 0
    
    
    for y_base in xrange(100,250,1):
        #y_base = 150 #20 # this represents 3" in front of the car
        index_y = height - y_base
        index_x = (width)//2     
        if index_x >= width:
            index_x = width - 1
        if index_x < 0:
            index_x = 0 
        x_left, x_right = find_offset_in_lane(edges, index_x, index_y, width)  
        midpoint = (x_right + x_left)//2
        
        if y_base == 100:
            pts0 = np.array([ (x_left,index_y), (x_right,index_y)], dtype=np.int32)

        
        if y_base % 2 == 0:
            pts = np.array([  (x_left,index_y), (x_right,index_y)], dtype=np.int32)
        else: 
            pts = np.array([ (x_right,index_y), (x_left,index_y)], dtype=np.int32)
        
        if y_base == 100:
            offset = midpoint - width//2
            midpoint_ref = midpoint
            index_y_ref = index_y


        pts0 = np.concatenate((pts0, pts))
        
        cv2.circle(img, (x_right, index_y), 3, (0,255,255), -1)
        cv2.circle(img, (x_left, index_y), 3, (0,255,255), -1)

    
    
    cv2.fillPoly(img, [pts0], (0,255, 0))
    fontFace = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, str(offset), (midpoint_ref, 255), fontFace, .5,(0,0,255))
    if offset > 0:
        cv2.arrowedLine(img, (midpoint_ref+40, 250), (midpoint_ref+90, 250), (255,0,0), 2,8, 0,0.5)
    elif offset < 0:
        cv2.arrowedLine(img, (midpoint_ref-10, 250), (midpoint_ref-60, 250), (255,0,0), 2,8, 0,0.5)
    else: 
        cv2.arrowedLine(img,  (midpoint_ref, 230), (midpoint_ref, 180), (255,0,0), 2,8, 0,0.5)
            
        ## Make Steering Calculations
    angle_adjacent = 20; #experimentally determined
    
    cv2.line(img, (midpoint_ref, index_y_ref-35),(midpoint_ref, index_y_ref+35), (255,255,255),3)
    cv2.line(img, (width//2, index_y_ref-5-35),(width//2, index_y_ref-5+35), (0,0,0),3)

    #cv2.circle(img, (midpoint_ref, index_y_ref), 3, (0,255,255),-1)
    #cv2.circle(img, (width//2, index_y_ref-5), 3, (0,0,255),-1)

        ## Publish vehicle steering directions 
        #self.offset_lane_pub.publish(Float32(offset))
        
    return offset 


def main(args):
    rospy.on_shutdown(shutdown_func)
    global image_processor_global
    global offset_global
    
    rospy.init_node('talker', anonymous=True)

    image_processor_global = image_processor()
    #rospy.init_node('image_processing', anonymous=True)
     
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


    
if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
