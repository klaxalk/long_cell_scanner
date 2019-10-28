#!/usr/bin/env python

import rospy
import time
import copy
import numpy

from gclib_ros.msg import Position
from rospix.msg import Image as RospixImage
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

# OpenCV + ROS cv_bridge
import cv2
from cv_bridge import CvBridge, CvBridgeError

class ImageStitcher:

    def mainTimer(self, event):

        rospy.loginfo_once('Main timer spinning')

        if not self.image_initialized and self.got_image_width and self.got_image_height:

            rospy.loginfo('initializing image')

            rospy.loginfo('px_width: {}'.format(self.px_width))
            rospy.loginfo('px_height: {}'.format(self.px_height))

            self.np_image = numpy.zeros((self.px_height, self.px_width), numpy.uint16)

            self.image_initialized = True

        if self.image_initialized:
          
            temp_image = copy.deepcopy(self.np_image)

            if numpy.count_nonzero(temp_image) > 0:
                image_empty = False
            else:
                image_empty = True

            # equalize the image histogram
            hist,bins = numpy.histogram(temp_image.flatten(),65536,[0,65535])
            cdf = hist.cumsum()
            cdf_normalized = cdf * hist.max()/ cdf.max()
            cdf_m = numpy.ma.masked_equal(cdf,0)
            cdf_m = (cdf_m - cdf_m.min())*65535/(cdf_m.max()-cdf_m.min())
            cdf = numpy.ma.filled(cdf_m,0).astype('uint16')
            img2 = cdf[temp_image]

            # convert the image to 8bit range
            img2 = img2.astype(numpy.float)
            if image_empty:
                img_8bit = 255 - 255*img2/1.0
            else:
                img_8bit = 255 - 255*img2/numpy.max(img2)
            img_8bit = img_8bit.astype(numpy.uint16)
            img_8bit = img_8bit.astype(numpy.uint8)

            # upscale the image to higher resolution
            upscale_factor = 2
            upscaled = cv2.resize(img_8bit, dsize=(self.px_width*upscale_factor, self.px_height*upscale_factor), interpolation = cv2.INTER_AREA)

            # convert the image to colored format
            upscaled = cv2.cvtColor(upscaled, cv2.COLOR_GRAY2BGR)

            # convert the image to ROS image
            image_message = self.bridge.cv2_to_imgmsg(upscaled, encoding="rgb8")

            # publish the image
            self.publisher_image.publish(image_message)

            rospy.loginfo('publishing')

    def callbackImage(self, data):

        self.image = data

        self.got_image = True

        if not self.got_fov:
            rospy.loginfo('waiting for fov')
            return

        if not self.got_image_width:
            rospy.loginfo('waiting for image width')
            return

        if not self.got_image_height:
            rospy.loginfo('waiting for image height')
            return

        if not self.got_position:
            rospy.loginfo('waiting for position')
            return

        if not self.image_initialized:
            rospy.loginfo('image not initialized')
            return

        rospy.loginfo('integrating image')

        for i,pixel in enumerate(data.image):

            x = i // 256
            y = i % 256

            image_x = int(x) + int(256*(((self.image_height/self.fov) - 1.0)/2.0 + self.position[1]/self.fov))
            image_y = int(y) + int(256*(((self.image_width/self.fov) - 1.0)/2.0 + self.position[0]/self.fov))

            self.np_image[image_x, image_y] += pixel

    def callbackPosition(self, data):

        self.position = data.position

        self.got_position = True

    def callbackFov(self, data):

        self.fov = data.data

        self.got_fov = True

    def callbackImageWidth(self, data):

        if not self.got_fov:
            return 

        self.image_width = data.data
        self.px_width = int(256.0*data.data/self.fov)

        self.got_image_width = True

    def callbackImageHeight(self, data):

        if not self.got_fov:
            return 

        self.image_height = data.data
        self.px_height = int(256.0*data.data/self.fov)

        self.got_image_height = True

    def __init__(self):

        rospy.init_node('image_stitcher', anonymous=True)

        # initialize the ROS cv_bridge
        self.bridge = CvBridge()

        self.got_position = False
        self.got_image = False
        self.got_fov = False
        self.got_image_width = False
        self.got_image_height = False

        self.px_width = 0
        self.px_height = 0

        self.position = []
        self.fov = 0
        self.image_size = 0

        self.image_initialized = False

        rospy.Subscriber("~image_width_in", Float64, self.callbackImageWidth, queue_size=1)
        rospy.Subscriber("~image_height_in", Float64, self.callbackImageHeight, queue_size=1)
        rospy.Subscriber("~image_fov_in", Float64, self.callbackFov, queue_size=1)
        rospy.Subscriber("~position_in", Position, self.callbackPosition, queue_size=1)

        rospy.Subscriber("~image_in", RospixImage, self.callbackImage, queue_size=1)

        # publishers
        self.publisher_image = rospy.Publisher("~image_out", Image, queue_size=1)

        rospy.Timer(rospy.Duration(2.0), self.mainTimer)

        rospy.spin()

if __name__ == '__main__':
    try:
        image_stitcher = ImageStitcher()
    except rospy.ROSInterruptException:
        pass
