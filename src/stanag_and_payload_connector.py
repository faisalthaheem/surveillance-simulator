#!/usr/bin/python3

import logging
import rospy
import std_msgs

import sys, traceback
import asyncio

from stanag4586vsm.stanag_server import *

FORMAT = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
logging.basicConfig(format=FORMAT)

logger = logging.getLogger("StanagServer")
logger.setLevel(logging.DEBUG)

# the publisher to send ptz message through to payload control node
pub_ptz = None

#the currently running loop
current_loop = None

async def publish_ptz_target(msg):
    pub_ptz.publish(stdmsgs.msg.String("hello " + msg))

def handle_message(wrapper, msg):
    logger.info("Got message in stanag_to_ros_iface [{:x}]".format(wrapper.message_type))
    current_loop.call_soon_threadsafe(publish_ptz_target, "faisal")

async def start_stanag_iface():

    logger.debug("Creating server")
    server = StanagServer(logging.DEBUG)

    await server.setup_service(current_loop)

    #set our callback to start getting requests unprocessed by default implementation
    server.get_entity("eo").set_callback_for_unhandled_messages(handle_message)

    logger.info("STANAG iface active")

    while rospy.is_shutdown() is False:
        logger.debug("Sleeping in stanag loop")
        
        await asyncio.sleep(1.0)

    logger.info("STANAG iface exiting")

async def main():

    global pub_ptz
    global current_loop

    # save for callbacks
    current_loop = asyncio.get_running_loop()

    try:
        logger.info("Begin init")

        rospy.init_node('stanag_and_payload_connector')

        logger.info("Creating publishers and subscribers")
        pub_ptz = rospy.Publisher('ptz_targets', std_msgs.msg.String, queue_size=10)

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