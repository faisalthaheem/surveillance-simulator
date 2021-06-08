import rospy
from sensor_msgs.msg import JointState

import signal
import sys
import threading
import time
import os

# --------------------------------------------------------------------------- #
# configure logging
# --------------------------------------------------------------------------- #
import logging
FORMAT = ('%(asctime)-15s %(threadName)-15s'
          ' %(levelname)-8s %(module)-15s:%(lineno)-8s %(message)s')
logging.basicConfig(format=FORMAT)
log = logging.getLogger()
log.setLevel(logging.DEBUG)

thread_ros_joint_state_publisher = None

class RosPublisher:
    def __init__(self):
        self.opening = False
        
    def joint_state_publisher(self):
        pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        rospy.init_node('joint_state_publisher')
        rate = rospy.Rate(10) # 10hz
        state = JointState()
        state.name=['base_to_mast','mast_to_mast_head','mast_head_to_pitch']
        state.position= [0.0,0.0,0.0]

        pos_end = 1.42
        pos = 0.0

        while not rospy.is_shutdown():
            state.header.stamp = rospy.Time.now()

            # if self.opening == True:  
            #     if pos < pos_end:
            #         pos += 0.005
            #         state.position = [pos]
            # else:
            #     #if pos > 0.1:
            #     print("pub=", pos)
            #     pos -= 0.005
            #     state.position = [pos]
            
            print("pub=", pos)
            pos -= 0.005
            state.position = [0.0, 0.0, pos]
            pub.publish(state)
            rate.sleep()

def signal_handler(sig, frame):
    print("\n\nSIGINT caught. Terminating..\n\n")
    os._exit(1)
    
if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    publisher = RosPublisher()

    publisher.joint_state_publisher()