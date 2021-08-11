#!/usr/bin/python3

#inspired by http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
__author__ =  'Faisal Thaheem <10938481+faisalthaheem@users.noreply.github.com>'
__version__=  '0.1'
__license__ = 'GPLV3'

#standard imports
import sys, time

#for img processing
import numpy as np
import cv2

#ros msgs
import rospy
from sensor_msgs.msg import CompressedImage

#gstreamer imports
import gi
gi.require_version('Gst','1.0')
gi.require_version('GstVideo','1.0')
from gi.repository import GObject, Gst, GstVideo

from fractions import Fraction

appsrc = None

DEBUG = False

class CameraImage:

    def __init__(self, topic="/payload/eo_cam/image_raw/compressed"):
        self.img_sub = rospy.Subscriber(topic, CompressedImage, self.cb_img_available, queue_size=1)

        FPS = Fraction(10)

        self.pts = 0
        self.duration = 10**9 / (FPS.numerator / FPS.denominator)


    def cb_img_available(self, ros_data):

        if appsrc is None:
            return

        np_arr = np.frombuffer(ros_data.data, np.uint8)
        mat_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        gst_buffer = Gst.Buffer.new_wrapped(mat_img.tobytes())

        self.pts += self.duration
        gst_buffer.pts = self.pts
        gst_buffer.duration = self.duration

        appsrc.emit("push-buffer", gst_buffer)

        if DEBUG is True:
            cv2.imshow('cv_img', mat_img)
            cv2.waitKey(2)

    def ndarray_to_gst_buffer(self, array):
            return Gst.Buffer.new_wrapped(array.tobytes())

def main(args):
    
    global appsrc

    ci = CameraImage()
    rospy.init_node('eo_img_processor', anonymous=True)

    Gst.init(None)

    #for nvidia h/w acceleration need libnvidia-encode-460 libnvidia-decode-460 libdrm-dev

    pipeline = Gst.parse_launch("appsrc format=GST_FORMAT_TIME name=appsrc emit-signals=True is-live=True block=True caps=\"video/x-raw,format=BGR,width=800,height=600,framerate=10/1\" ! queue ! videoconvert ! x264enc tune=zerolatency ! rtph264pay ! udpsink host=127.0.0.1 port=5000")
    # Todo: enable h/w acceleartion on nvidia, intel and amd gpus
    # nvidia - https://www.endpoint.com/blog/2021/04/gstreamer-nvenc-for-ubuntu-20-04/
    # pipeline = Gst.parse_launch("appsrc format=GST_FORMAT_TIME name=appsrc emit-signals=True is-live=True block=True caps=\"video/x-raw,format=BGR,width=800,height=600,framerate=10/1\" ! queue ! videoconvert ! nvh264enc tune=zerolatency ! rtph264pay ! udpsink host=127.0.0.1 port=5000")
    
    pipeline.set_state(Gst.State.PLAYING)

    appsrc = pipeline.get_by_name("appsrc")

    try:
        rospy.spin()    
    except KeyboardInterrupt:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)