#!/usr/bin/python3

#inspired by http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
__author__ =  'Faisal Thaheem <10938481+faisalthaheem@users.noreply.github.com>'
__version__=  '0.1'
__license__ = 'GPLV3'

import sys, time

import numpy as np

import cv2

import roslib
import rospy

from sensor_msgs.msg import CompressedImage

class CameraImage:
    def __init__(self, topic="/payload/eo_cam/image_raw/compressed"):
        self.img_sub = rospy.Subscriber(topic, CompressedImage, self.cb_img_available, queue_size=1)

    def cb_img_available(self, ros_data):

        # print(ros_data.height, ros_data.width)

        np_arr = np.fromstring(ros_data.data, np.uint8)
        # print(np_arr.shape)
        mat_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # rospy.loginfo("HxW: [{}]x[{}]".format(mat_img.rows, mat_img.cols))

        cv2.imshow('cv_img', mat_img)
        cv2.waitKey(2)

def main(args):
    ci = CameraImage()
    rospy.init_node('eo_img_processor', anonymous=True)
    
    try:
        rospy.spin()    
    except KeyboardInterrupt:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)