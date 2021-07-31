#!/usr/bin/python3

from os import getenv
from gazebo_msgs.srv import GetLinkState, SetLinkState, ApplyJointEffort
from gazebo_msgs.msg import LinkState
import rospy
import math
import copy
import tf.transformations
from simple_pid import PID


#for constants
from stanag4586edav1.message20030 import Message20030


from surveillance_simulator.msg import Ptz, RelativePanTilt, MastCommand

get_link_state_proxy = None
set_link_state_proxy = None
apply_joint_effort_proxy = None

#eo related
target_centreline_azimuth_angle = 0.0
current_centerline_azimuth_angle = 0.0
target_centerline_elevation_angle = 1.57
current_centerline_elevation_angle = 0.0

relative_pan_tilt = RelativePanTilt()

#mast related
#Todo, get these limits from joints
"""Comes from model"""
MAST_MIN = 1
"""See model for limits"""
MAST_MAX = 3
MAST_STEP = 0.02

mast_height_target = 1.0
pid_mast = PID(50.0, 50.0, 10.0, setpoint=mast_height_target)


def set_ptz_callback(ptz):

    global target_centreline_azimuth_angle
    global target_centerline_elevation_angle

    target_centreline_azimuth_angle = ptz.pan
    target_centerline_elevation_angle = ptz.tilt

def set_relative_pan_tilt_callback(msg):

    global relative_pan_tilt

    relative_pan_tilt = copy.copy(msg)

def mast_command_callback(msg):
    global mast_height_target

    if msg.command_type == Message20030.CMD_TYPE_MOVE_ABSOLUTE:
        mast_height_target = msg.absolute_height
    else:
        if msg.command_type == Message20030.CMD_TYPE_MOVE_UP and mast_height_target + MAST_STEP < MAST_MAX:
            mast_height_target += MAST_STEP
        elif msg.command_type == Message20030.CMD_TYPE_MOVE_DOWN and mast_height_target - MAST_STEP > MAST_MIN:
            mast_height_target -= MAST_STEP

    pid_mast.setpoint = mast_height_target


def main():
    try:

        global get_link_state_proxy
        global set_link_state_proxy
        global apply_joint_effort_proxy

        isRelativePanTiltMode = True if getenv("PAN_TILT_MODE") == "relative" else False

        rospy.init_node('payload_control_iface')

        rate = rospy.Rate(10) #todo read from config

        rospy.loginfo("Init rospy iface")

        rospy.wait_for_service('/gazebo/get_link_state')
        get_link_state_proxy = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

        rospy.wait_for_service('/gazebo/set_link_state')
        set_link_state_proxy = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)

        rospy.wait_for_service('/gazebo/apply_joint_effort')
        apply_joint_effort_proxy = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)

        rospy.Subscriber("ptz_targets", Ptz, set_ptz_callback)
        rospy.Subscriber("relative_pan_tilt", RelativePanTilt, set_relative_pan_tilt_callback)
        rospy.Subscriber("mast_command", MastCommand, mast_command_callback)

        #http://docs.ros.org/en/diamondback/api/gazebo/html/srv/GetLinkState.html
        while not rospy.is_shutdown():
            
            mast_state = get_link_state_proxy("pl1::mast", "")

            err = mast_state.link_state.pose.position.z
            force = pid_mast(err)

            res = apply_joint_effort_proxy(
                'base_to_mast', 
                force, 
                rospy.Time(0.0), 
                rospy.Duration.from_sec(0.1)
            )


            if isRelativePanTiltMode is False:
                yaw_slip_ring_state = get_link_state_proxy("pl1::slip_ring_yaw", "")
                camera_tilt = get_link_state_proxy("pl1::camera", "")

                if yaw_slip_ring_state.success:

                    q = yaw_slip_ring_state.link_state.pose.orientation
                    rpy = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
                    current_centerline_azimuth_angle = rpy[2]
                    # rospy.loginfo("current_centerline_azimuth_angle = {}".format(math.degrees(current_centerline_azimuth_angle)))

                    force = 0.0
                    if abs(current_centerline_azimuth_angle - target_centreline_azimuth_angle) > float(0.017):
                        if current_centerline_azimuth_angle > target_centreline_azimuth_angle:
                            force = -2.0
                        else:
                            force = 2.0

                    if force != 0.0:
                        res = apply_joint_effort_proxy('mast_slip_ring_yaw', force, rospy.Time(0.0), rospy.Duration.from_sec(0.1))

                if camera_tilt.success:

                    q = camera_tilt.link_state.pose.orientation
                    rpy = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
                    current_centerline_elevation_angle = rpy[0]

                    force = 0.0
                    if abs(current_centerline_elevation_angle - target_centerline_elevation_angle) > float(0.017):
                        if current_centerline_elevation_angle > target_centerline_elevation_angle:
                            force = -2.0
                        else:
                            force = 2.0
                            
                    if force != 0.0:
                        res = apply_joint_effort_proxy('slip_ring_camera_tilt', force, rospy.Time(0.0), rospy.Duration.from_sec(0.1))
            else:

                if relative_pan_tilt.pan_direction != 0:
                    
                    force = 1.0 + relative_pan_tilt.pan_force
                    if relative_pan_tilt.pan_direction > 0:
                        force *= -1.0
                    
                    res = apply_joint_effort_proxy('mast_slip_ring_yaw', force, rospy.Time(0.0), rospy.Duration.from_sec(0.1))
                    
                if relative_pan_tilt.tilt_direction != 0:
                    
                    force = 1.0 + relative_pan_tilt.tilt_force
                    if relative_pan_tilt.tilt_direction < 0:
                        force *= -1.0
                    
                    res = apply_joint_effort_proxy('slip_ring_camera_tilt', force, rospy.Time(0.0), rospy.Duration.from_sec(0.1))

            rate.sleep()

        #clean up
        apply_joint_effort_proxy.close()
        get_link_state_proxy.close()
        set_link_state_proxy.close()

    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed:  {0}".format(e))

    rospy.loginfo("Shutdown rospy iface")
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass