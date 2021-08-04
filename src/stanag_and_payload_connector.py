#!/usr/bin/python3

import os
import logging
import rospy
import copy
import sys, traceback
import asyncio
import json

from stanag4586vsm.stanag_server import *
from stanag4586edav1.message_wrapper import *
from stanag4586edav1.message200 import *
from stanag4586edav1.message20010 import *
from stanag4586edav1.message20020 import *
from stanag4586edav1.message20040 import *
from stanag4586edav1.message21 import *

from surveillance_simulator.msg import Ptz, RelativePanTilt, MastCommand, MastStatus

FORMAT = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
logging.basicConfig(format=FORMAT)

logger = logging.getLogger("StanagServer")
logger.setLevel(logging.DEBUG)

# the publisher to send ptz message through to payload control node
to_topic_ptz = None
to_topic_relative_pt = None
to_topic_mast_command = None

#our local data storage
ptz_cache = Ptz()

#the currently running loop
current_loop = None

#the stanag server
server = None

#config variables read through env
EXTERNAL_RTSP_IP_ADDRESS = os.environ.get("EXTERNAL_RTSP_IP_ADDRESS","localhost")


def mast_status_callback(msg):

    if server is None: return
    
    station = server.get_entity("mast")
    if station is None:
        logger.debug("Station is none. Cannot get mast details.") 
        return

    monitoring_cucs = station.getMonitoringCucs()

    msg20040 = Message20040(Message20040.MSGNULL)

    msg20040.time_stamp = 0x00
    msg20040.vehicle_id = station.getVehicleId()
    msg20040.station_number = station.getStationId()
    msg20040.min_height = msg.min_height
    msg20040.max_height = msg.max_height
    msg20040.current_height = msg.current_height
    msg20040.error_code = msg.error_code

    for cucsid in monitoring_cucs:

        msg20040.cucs_id = cucsid
    
        wrapped_reply = MessageWrapper(MessageWrapper.MSGNULL)
        wrapped_reply = wrapped_reply.wrap_message(1, 20040, msg20040, False)

        current_loop.call_soon(server.tx_data, wrapped_reply)
    
    logger.debug("Exit Published mast status") 

async def process_message(wrapper, msg):

    global ptz_cache
    
    #to keep track of when we need to publish
    msg_to_publish = None

    if wrapper.message_type == 200:
        ptz_cache.pan = float(msg.set_centreline_azimuth_angle)
        ptz_cache.tilt = float(msg.set_centreline_elevation_angle)

        msg_to_publish = copy.copy(ptz_cache)

        logger.debug("On MSG_200 Setting Pan [{}] and Tilt [{}]".format(msg_to_publish.pan, msg_to_publish.tilt))
        to_topic_ptz.publish(msg_to_publish)
    
    elif wrapper.message_type == 20000:

        rpt = RelativePanTilt()
        rpt.pan_force = float(msg.pan_force)
        rpt.pan_direction = msg.pan_direction
        rpt.tilt_force = float(msg.tilt_force)
        rpt.tilt_direction = msg.tilt_direction

        msg_to_publish = rpt

        logger.debug("On MSG_20000 Setting pan_force [{}] pan_direction [{}] tilt_force [{}] tilt_direction [{}]".format(
            rpt.pan_force,
            rpt.pan_direction,
            rpt.tilt_force,
            rpt.tilt_direction,
        ))
        to_topic_relative_pt.publish(msg_to_publish)

    elif wrapper.message_type == 20010:
        #todo - handle station for responding with correct config
        msg20020 = Message20020(Message20020.MSGNULL)
    
        msg20020.time_stamp = 0x00
        msg20020.vehicle_id = msg.vehicle_id
        msg20020.cucs_id = msg.cucs_id
        msg20020.station_number = msg.station_number
        msg20020.requested_query_type = msg.query_type
        msg20020.set_response(json.dumps({"daylight":"rtsp://{}:8554/hd".format(EXTERNAL_RTSP_IP_ADDRESS)}))

        wrapped_reply = MessageWrapper(MessageWrapper.MSGNULL)
        wrapped_reply = wrapped_reply.wrap_message(wrapper.msg_instance_id, 20020, msg20020, False)

        current_loop.call_soon(server.tx_data, wrapped_reply)
    
    elif wrapper.message_type == 20030:

        mastCmd = MastCommand()
        mastCmd.command_type = int(msg.command_type)
        mastCmd.absolute_height = float(msg.absolute_height)

        logger.debug("On MSG_20030 command_type [{}] absolute_height [{}]".format(
            mastCmd.command_type,
            mastCmd.absolute_height
        ))
        to_topic_mast_command.publish(mastCmd)


def handle_message(wrapper, msg):
    logger.debug("Got message in stanag_to_ros_iface [{}]".format(wrapper.message_type))
    
    #asyncio limitation
    current_loop.create_task(process_message(wrapper, msg))

async def start_stanag_iface():

    global server

    logger.debug("Creating server")
    server = StanagServer(logging.DEBUG)

    await server.setup_service(current_loop, StanagServer.MODE_VEHICLE, Message21.VEHICLE_TYPE_UGV, Message21.UGV_SUB_TYPE_SURV)

    #set our callback to start getting requests unprocessed by default implementation
    server.get_entity("eo").set_callback_for_unhandled_messages(handle_message)

    logger.info("STANAG iface active")

    while rospy.is_shutdown() is False:
        
        #run event loops for both frameworks, 50ms in total
        await asyncio.sleep(0.025)
        rospy.sleep(0.025)

    logger.info("STANAG iface exiting")

async def main():

    global to_topic_ptz
    global to_topic_relative_pt
    global to_topic_mast_command
    global current_loop

    # save for callbacks
    current_loop = asyncio.get_running_loop()

    try:
        logger.info("Begin init")

        rospy.init_node('stanag_and_payload_connector')

        logger.info("Creating publishers and subscribers")
        to_topic_ptz = rospy.Publisher('/pl1/ptz_targets', Ptz, queue_size=10)
        to_topic_relative_pt = rospy.Publisher('/pl1/relative_pan_tilt', RelativePanTilt, queue_size=10)
        to_topic_mast_command = rospy.Publisher('/pl1/mast_command', MastCommand, queue_size=10)

        #subscribe to statuses
        rospy.Subscriber("/pl1/mast_status", MastStatus, mast_status_callback)

        rospy.sleep(1.0) #establish connection

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