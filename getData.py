#!/usr/bin/env python
"""OpenCV feature detectors with ros CompressedImage Topics in python.

This example subscribes to a ros topic containing sensor_msgs 
CompressedImage. It converts the CompressedImage into a numpy.ndarray, 
then detects and marks features in that image. It finally displays 
and publishes the new image - again as CompressedImage topic.
"""

# Python libs
import sys, time

# numpy and scipy
import numpy as np
from PIL import Image as I
from PIL import Image
from scipy.ndimage import filters

# OpenCV
import cv2, array
# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import Image
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

VERBOSE=False

class image_feature:

    def __init__(self):

        self.subscriber = rospy.Subscriber("/image",
                                           Image, self.callback,  queue_size = 1)
        if VERBOSE :
            print "subscribed to /image"


    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print 'received image of type: "%s"' % ros_data.format

        #### direct conversion to CV2 ####
        #np_arr = np.fromstring(ros_data.data, np.uint8)
        from PIL import Image
        np_arr = ros_data.data
        image_byte_array = array.array('b', np_arr)
        image_buffer = I.frombuffer("RGB", (640,480), image_byte_array, "raw", "BGR", 0, 1)
        image_buffer  = image_buffer.transpose(Image.FLIP_LEFT_RIGHT)
        image_buffer  = image_buffer.transpose(Image.ROTATE_180)

        img2 = np.asarray(image_buffer)
        #image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

        #### Feature detectors using CV2 #### 
        # "","Grid","Pyramid" + 
        # "FAST","GFTT","HARRIS","MSER","ORB","SIFT","STAR","SURF"


        # method = "GridFAST"
        # feat_det = cv2.FeatureDetector_create(method)
        # time1 = time.time()
        #
        # # convert np image to grayscale
        # featPoints = feat_det.detect(
        #image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
        #image_np =  np.reshape(np_arr, (640, 480, 3))
        #print(image_np.shape)
        #image_np = cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR)
        time2 = time.time()

        if VERBOSE :
            print '%s detector found: %s points in: %s sec.'%(method,
                                                              len(featPoints),time2-time1)

        # for featpoint in featPoints:
        #     x,y = featpoint.pt
        #     cv2.circle(image_np,(int(x),int(y)), 3, (0,0,255), -1)

        #cv2.imshow('cv_img', image_np)
        #cv2.waitKey(2)
        cv2.imwrite(str(time2)+'.png',img2)

def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)