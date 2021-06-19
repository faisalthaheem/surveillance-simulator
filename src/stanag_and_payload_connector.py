#!/usr/bin/python3

import logging
import rospy
import copy
import sys, traceback
import asyncio

from stanag4586vsm.stanag_server import *
from stanag4586edav1.message_wrapper import *
from stanag4586edav1.message200 import *

from surveillance_simulator.msg import Ptz

FORMAT = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
logging.basicConfig(format=FORMAT)

logger = logging.getLogger("StanagServer")
logger.setLevel(logging.DEBUG)

# the publisher to send ptz message through to payload control node
to_topic_ptz = None

#our local data storage
ptz_cache = Ptz()

#the currently running loop
current_loop = None

async def publish_ptz_target(wrapper, msg):

    global ptz_cache
    
    #to keep track of when we need to publish
    msg_to_publish = None

    if wrapper.message_type == 0x200:
        ptz_cache.pan = msg.set_centreline_azimuth_angle
        ptz_cache.tilt = msg.set_centreline_elevation_angle

        msg_to_publish = copy.copy(ptz_cache)

    if msg_to_publish is not None:
        to_topic_ptz.publish(msg_to_publish)


def handle_message(wrapper, msg):
    logger.debug("Got message in stanag_to_ros_iface [{:x}]".format(wrapper.message_type))
    
    #asyncio limitation
    current_loop.create_task(publish_ptz_target(wrapper, msg))

async def start_stanag_iface():

    logger.debug("Creating server")
    server = StanagServer(logging.DEBUG)

    await server.setup_service(current_loop)

    #set our callback to start getting requests unprocessed by default implementation
    server.get_entity("eo").set_callback_for_unhandled_messages(handle_message)

    logger.info("STANAG iface active")

    while rospy.is_shutdown() is False:
        
        #run event loops for both frameworks, 50ms in total
        await asyncio.sleep(0.25)
        rospy.sleep(0.25)

    logger.info("STANAG iface exiting")

async def main():

    global to_topic_ptz
    global current_loop

    # save for callbacks
    current_loop = asyncio.get_running_loop()

    try:
        logger.info("Begin init")

        rospy.init_node('stanag_and_payload_connector')

        logger.info("Creating publishers and subscribers")
        to_topic_ptz = rospy.Publisher('ptz_targets', Ptz, queue_size=10)
        rospy.sleep(0.0) #establish connection

        logger.info("Starting stanag interface")
        await start_stanag_iface()

        logger.info("End init")

        logger.info("Program end.")

    except Exception:
        logger.error("Unhandled error: [{}]".format(sys.exc_info()[0]))
        print("-"*60)
        traceback.print_exc(file=sys.stdout)
        print("-"*60)

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except:
        pass