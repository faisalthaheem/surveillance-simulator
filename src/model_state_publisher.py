#!/usr/bin/python3

from gazebo_msgs.srv import GetLinkState
import rospy

from surveillance_simulator.msg import MastStatus

#services
get_link_state_proxy = None

#topics
to_topic_mast_status = None

#mast related
#Todo, get these limits from joints
"""Comes from model"""
MAST_MIN = 1
"""See model for limits"""
MAST_MAX = 3

def main():
    try:

        global get_link_state_proxy
        global to_topic_mast_status

        rospy.init_node('model_state_publisher')

        rate = rospy.Rate(5) #every 200ms

        rospy.loginfo("Init rospy iface")

        #get references to services
        rospy.wait_for_service('/gazebo/get_link_state')
        get_link_state_proxy = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

        #init comms
        to_topic_mast_status = rospy.Publisher('mast_status', MastStatus, queue_size=10)

        #http://docs.ros.org/en/diamondback/api/gazebo/html/srv/GetLinkState.html
        while not rospy.is_shutdown():
            
            mast_state = get_link_state_proxy("pl1::extendable_mast::em_mast", "")

            mastStatus = MastStatus()
            mastStatus.min_height = MAST_MIN
            mastStatus.max_height = MAST_MAX
            mastStatus.error_code = 0 # no error
            mastStatus.current_height = mast_state.link_state.pose.position.z

            to_topic_mast_status.publish(mastStatus)

            rate.sleep()

        #clean up
        get_link_state_proxy.close()

    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed:  {0}".format(e))

    rospy.loginfo("Shutdown rospy iface")
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass