#!/usr/bin/python3


from gazebo_msgs.srv import ApplyJointEffort
import rospy

def main():
    try:

        rospy.init_node('payload_control_iface')

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

    rospy.loginfo("Shutdown rospy iface")
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass