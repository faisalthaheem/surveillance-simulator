#!/usr/bin/python3

from os import link
from gazebo_msgs.srv import GetLinkState, SetLinkState, ApplyJointEffort
from gazebo_msgs.msg import LinkState
import rospy
import math
import copy
import tf.transformations

from surveillance_simulator.msg import Ptz

get_link_state_proxy = None
set_link_state_proxy = None
apply_joint_effort_proxy = None

target_centreline_azimuth_angle = 0.0
current_centerline_azimuth_angle = 0.0

def set_ptz_callback(ptz):

    global target_centreline_azimuth_angle

    target_centreline_azimuth_angle = ptz.pan



def main():
    try:

        global get_link_state_proxy
        global set_link_state_proxy
        global apply_joint_effort_proxy

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

        while not rospy.is_shutdown():


            slip_ring_state = get_link_state_proxy("pl1::slip_ring_yaw", "")

            if slip_ring_state.success:

                q = slip_ring_state.link_state.pose.orientation
                rpy = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
                current_centerline_azimuth_angle = rpy[2]
                rospy.loginfo("current_centerline_azimuth_angle = {}".format(math.degrees(current_centerline_azimuth_angle)))

                force = 0.0
                if abs(current_centerline_azimuth_angle - target_centreline_azimuth_angle) > 0.017:
                    if current_centerline_azimuth_angle > target_centreline_azimuth_angle:
                        force = -2.0
                    else:
                        force = 2.0

                    if force != 0.0:
                        res = apply_joint_effort_proxy('mast_slip_ring_yaw', force, rospy.Time(0.0), rospy.Duration.from_sec(0.1))


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