#!/usr/bin/python3

from gazebo_msgs.srv import GetModelState, ApplyJointEffort
import rospy
import math

from simple_pid import PID

from surveillance_simulator.msg import Ptz


set_point_pan = 0.0

def set_ptz_callback(ptz):
    global set_point_pan

    set_point_pan = ptz.pan

    rospy.loginfo("set_point_pan set to [{}]".format(set_point_pan))

def main():
    try:

        rospy.init_node('payload_control_iface')

        rate = rospy.Rate(2) #todo read from config

        rospy.loginfo("Init rospy iface")

        rospy.wait_for_service('/gazebo/get_model_state')
        gms_proxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        rospy.wait_for_service('/gazebo/apply_joint_effort')
        aje_proxy = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)

        pid_pan = PID(1, 0.1, 0.05, setpoint=set_point_pan)

        rospy.Subscriber("ptz_targets", Ptz, set_ptz_callback)

        while not rospy.is_shutdown():
            model_state = gms_proxy("pl1","slip_ring_yaw")
            # rospy.loginfo("Value of x : " + str(model_state.pose.position.x))
            # rospy.loginfo("Value of y : " + str(model_state.pose.position.y))
            pid_pan.setpoint = set_point_pan
            effort = pid_pan(model_state.pose.orientation.z)
            res = aje_proxy('mast_slip_ring_yaw', effort, rospy.Time(0.0), rospy.Duration.from_sec(1))
            rospy.loginfo("Z [{}] Eff [{}]".format(str(math.degrees(model_state.pose.orientation.z)), effort))


            # res = aje_proxy('tbase_camera', 1.055, rospy.Time(0.0), rospy.Duration.from_sec(1))

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