#!/usr/bin/python3

import logging

from gazebo_msgs.srv import ApplyJointEffort
import rospy

import sys, traceback
import asyncio
import threading
import time

from stanag4586vsm.stanag_server import *

FORMAT = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
logging.basicConfig(format=FORMAT)

logger = logging.getLogger("StanagServer")
logger.setLevel(logging.DEBUG)

def handle_message(wrapper, msg):
    logger.info("Got message [{:x}]".format(wrapper.message_type))

async def start_stanag_iface():

    loop = asyncio.get_running_loop()

    logger.debug("Creating server")
    server = StanagServer(logging.DEBUG)

    await server.setup_service(loop)

    #set our callback to start getting requests unprocessed by default implementation
    server.get_entity("eo").set_callback_for_unhandled_messages(handle_message)

    logger.info("STANAG iface active")

    while True:
        await asyncio.sleep(1.0)

def start_stanag_iface_proxy():
    asyncio.run(start_stanag_iface())

def start_ros_nodes():
    
    try:

        rospy.init_node('payload_control_iface', anonymous=True)
        rate = rospy.Rate(1) #todo read from config

        rospy.loginfo("Init rospy iface")

        rospy.wait_for_service('/gazebo/apply_joint_effort')
        aje_proxy = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)

        while not rospy.is_shutdown():
            res = aje_proxy('tbase_camera', 1.10, rospy.Time(0.0), rospy.Duration.from_sec(1))

            res = aje_proxy('mast_slip_ring_yaw', 1.10, rospy.Time(0.0), rospy.Duration.from_sec(1))
            
            rate.sleep()

        aje_proxy.close()

    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed:  {0}".format(e))

def main():
    try:
        logger.info("Begin starting services")
        
        logger.info("Starting STANAG server")
        stanag_thread = threading.Thread(target = start_stanag_iface_proxy)
        stanag_thread.start()

        # logger.info("Starting ROS Node")
        start_ros_nodes()

        logger.info("Exit starting services. Program end.")


    except Exception:
        logger.error("Unhandled error: [{}]".format(sys.exc_info()[0]))
        print("-"*60)
        traceback.print_exc(file=sys.stdout)
        print("-"*60)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass