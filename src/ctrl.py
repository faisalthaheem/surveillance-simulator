from gazebo_msgs.srv import ApplyJointEffort
import rospy
import time

try:
    rospy.wait_for_service('/gazebo/apply_joint_effort')
    aje_proxy = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)

    res = aje_proxy('tbase_camera', 1.10, rospy.Time(0.0), rospy.Duration.from_sec(1))
    print(res)

    res = aje_proxy('mast_slip_ring_yaw', 1.10, rospy.Time(0.0), rospy.Duration.from_sec(1))
    print(res)

except rospy.ServiceException as e:
    rospy.loginfo("Service call failed:  {0}".format(e))
