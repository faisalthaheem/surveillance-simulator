#!/usr/bin/python3

from os import link
from gazebo_msgs.srv import GetLinkState, SetLinkState, ApplyJointEffort
from gazebo_msgs.msg import LinkState
import rospy
import math
import copy

from surveillance_simulator.msg import Ptz


get_link_state_proxy = None
set_link_state_proxy = None
apply_joint_effort_proxy = None

target_centreline_azimuth_angle = 0.0

def set_ptz_callback(ptz):

    global target_centreline_azimuth_angle

    target_centreline_azimuth_angle = ptz.pan



def main():
    try:

        global get_link_state_proxy
        global set_link_state_proxy
        global apply_joint_effort_proxy

        rospy.init_node('payload_control_iface')

        rate = rospy.Rate(30) #todo read from config

        rospy.loginfo("Init rospy iface")

        rospy.wait_for_service('/gazebo/get_link_state')
        get_link_state_proxy = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

        rospy.wait_for_service('/gazebo/set_link_state')
        set_link_state_proxy = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)

        rospy.wait_for_service('/gazebo/apply_joint_effort')
        apply_joint_effort_proxy = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)

        rospy.Subscriber("ptz_targets", Ptz, set_ptz_callback)

        while not rospy.is_shutdown():

            linkstate = get_link_state_proxy("pl1::slip_ring_yaw", "")
            if linkstate.success:

                if abs(linkstate.link_state.pose.orientation.z - target_centreline_azimuth_angle) > 0.02:
                
                    nustate = LinkState()
                    nustate = copy.copy(linkstate.link_state)

                    nustate.pose.orientation.z += 0.005

                    set_link_state_proxy(nustate)

            else:
                rospy.logerr("get_link_state_proxy call failed")


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