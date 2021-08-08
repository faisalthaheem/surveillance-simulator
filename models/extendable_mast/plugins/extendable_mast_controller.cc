#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>


#include <sdf/sdf.hh>

#include <gazebo/msgs/msgs.hh>
//https://github.com/arpg/Gazebo/blob/master/gazebo/physics/BoxShape.cc
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Subscriber.hh>
#include <gazebo/transport/Publisher.hh>

#include <surveillance_simulator/MastStatus.h>
#include <surveillance_simulator/MastCommand.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <string>
#include <sstream>

using namespace std;

typedef std::map< std::string, gazebo::physics::JointPtr > ModelJoints;

namespace gazebo
{
    class ExtendableMastController : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            // Store the pointer to the model
            this->model = _parent;
            this->sdf = _sdf;

            // Read parameters from sdf
            
            //Topic to which status messages are published to, composed as /modelname/<topic_name>
            std::string topic_name_pub_status;
            //Topic on which commands are recevied
            std::string topic_name_sub_cmds;
            //Rate at which status is published
            int publish_rate;

            std::string tmp_name_link;
            std::string tmp_name_joint;
            ignition::math::Vector3d tmp_vec3;
            

            if(!readFromSdf("topic_name_sub_cmds", topic_name_sub_cmds))
            {
                return;
            }
            topic_name_sub_cmds = this->model->GetName() + "/" + topic_name_sub_cmds;

            if(!readFromSdf("topic_name_pub_status", topic_name_pub_status))
            {
                return;
            }
            topic_name_pub_status = this->model->GetName() + "/" + topic_name_pub_status;

            if(!readFromSdf("link_name_segment_mount", tmp_name_link))
            {
                return;
            }
            link_seg_top_mount= model->GetLink(tmp_name_link);
            if(link_seg_top_mount == nullptr)
            {
                ROS_FATAL("Unable to find link [%s] in model.", tmp_name_link.c_str());
                return;
            }

            //https://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Joint.html
            //https://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/namespacegazebo_1_1physics.html#af7e4c06bce794eef119784b85ce330f5
            //https://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1JointController.html

            // ModelJoints available_joints = model->GetJointController()->GetJoints();
            // printAvailableJoints(available_joints);


            if(!readFromSdf("joint_lift_seg_1", tmp_name_joint))
            {
                return;
            }

            joint_lift_seg_1 = this->model->GetJoint(tmp_name_joint);
            if(!joint_lift_seg_1)
            {
                ROS_ERROR("Unable find joint [%s] in model", tmp_name_joint.c_str());
                return;
            }

            if(!readFromSdf("joint_lift_seg_2", tmp_name_joint))
            {
                return;
            }
            joint_lift_seg_2 = this->model->GetJoint(tmp_name_joint);
            if(!joint_lift_seg_2)
            {
                ROS_ERROR("Unable find joint [%s] in model", tmp_name_joint.c_str());
                return;
            }

            if(!readFromSdf("publish_rate", publish_rate))
            {
                return;
            }

            if(!readFromSdf("reported_min_height", this->reported_min_height))
            {
                return;
            }

            if(!readFromSdf("reported_max_height", this->reported_max_height))
            {
                return;
            }

            if(!readFromSdf("step_height", this->step_height))
            {
                return;
            }

            if(!readFromSdf("joint_lift_seg_1_pid", tmp_vec3))
            {
                return;
            }
            this->pid_joint_lift_seg_1 = common::PID(tmp_vec3.X(), tmp_vec3.Y(), tmp_vec3.Z());

            if(!readFromSdf("joint_lift_seg_2_pid", tmp_vec3))
            {
                return;
            }
            this->pid_joint_lift_seg_2 = common::PID(tmp_vec3.X(), tmp_vec3.Y(), tmp_vec3.Z());

            this->model->GetJointController()->SetPositionPID(
                this->joint_lift_seg_1->GetScopedName(), this->pid_joint_lift_seg_1);

            this->model->GetJointController()->SetPositionPID(
                this->joint_lift_seg_2->GetScopedName(), this->pid_joint_lift_seg_2);

            //Set initial target
            this->model->GetJointController()->SetPositionTarget(
                this->joint_lift_seg_1->GetScopedName(), this->joint_lift_seg_1->LowerLimit());
            this->model->GetJointController()->SetPositionTarget(
                this->joint_lift_seg_2->GetScopedName(), this->joint_lift_seg_2->LowerLimit());

            //http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.Writing_the_Subscriber_Node
            //Init ros related stuff
            if(!ros::isInitialized())
            {
                ROS_FATAL("ROS plugin not initalized.");
                return;
            }
            sub_cmd = node.subscribe(topic_name_sub_cmds, 10, &ExtendableMastController::onCmdReceived, this);
            pub_state = node.advertise<surveillance_simulator::MastStatus>(topic_name_pub_status, 10);

            publish_timer = node.createTimer(publish_rate, &ExtendableMastController::advertStatus, this);

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&ExtendableMastController::OnUpdate, this));

        }

        void printAvailableJoints(ModelJoints &available_joints)
        {
            ModelJoints::iterator itr = available_joints.begin();
            while(itr != available_joints.end())
            {
                ROS_INFO("Available joint [%s]", itr->first.c_str());
                ++itr;
            }
        }

        // Called by the world update start event
    public:
        void OnUpdate()
        {
            //https://answers.gazebosim.org//question/18816/how-correctly-move-joints-by-setpositionpid-and-setpositiontarget/
            this->model->GetJointController()->Update();
        }

        void onCmdReceived(const surveillance_simulator::MastCommand& cmd)
        {
            if(cmd.command_type == 30) //absolute
            {
                //not implemented
                //target_joint_displacement = cmd.absolute_height;
            }else{

                if(cmd.command_type == 10)
                {
                    if(setJointTarget(this->joint_lift_seg_1, target_joint_displacement + step_height))
                    {
                        target_joint_displacement += step_height;
                        setJointTarget(this->joint_lift_seg_2, target_joint_displacement);
                    }
                    
                }else if( cmd.command_type == 20)
                {
                    if(setJointTarget(this->joint_lift_seg_1, target_joint_displacement - step_height))
                    {
                        target_joint_displacement -= step_height;
                        setJointTarget(this->joint_lift_seg_2, target_joint_displacement);
                    }
                }
            }
        }

        /**
         * 
         * Will return true if the given target is within limits of the joint
         * 
         **/
        bool setJointTarget(gazebo::physics::JointPtr &joint, double target)
        {
            if( target >= joint->LowerLimit() && target <= joint->UpperLimit())
            {
                this->model->GetJointController()->SetPositionTarget(
                    joint->GetScopedName(), target);
                    return true;
            }
            return false;
        }

        void advertStatus(const ros::TimerEvent& e)
        {
             
            surveillance_simulator::MastStatus status;
            status.min_height = this->reported_min_height;
            status.max_height = this->reported_max_height;
            status.current_height = getMastHeight();
            status.error_code = 0;

            pub_state.publish(status);
            
        }

        double getMastHeight()
        {
            //https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1Link.html
            //https://osrf-distributions.s3.amazonaws.com/ign-math/api/1.0.0/classignition_1_1math_1_1Pose3.html
            ignition::math::Pose3d pose = link_seg_top_mount->RelativePose();
            ignition::math::Vector3 position = pose.Pos();

            // ROS_INFO("Mast relative pose is [%f] target is [%f]", position.Z(), this->target_joint_displacement);
           
            return position.Z();
        }

    private:
        template<typename T>
        inline bool readFromSdf(string element_name, T &store_to)
        {
            if(!this->sdf->HasElement(element_name))
            {
                ROS_FATAL("Missing [%s] element in sdf.", element_name.c_str());

                return false;
            }
            store_to = this->sdf->Get<T>(element_name);

            return true;
        }

        // Pointer to the model
    private:
        //https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1Model.html
        physics::ModelPtr model;
        
        gazebo::physics::JointPtr joint_lift_seg_1; 
        gazebo::physics::JointPtr joint_lift_seg_2;
        double target_joint_displacement = 0.0;

        gazebo::physics::LinkPtr link_seg_top_mount;

        sdf::ElementPtr sdf;

        ros::NodeHandle node;
        ros::Publisher pub_state;
        ros::Subscriber sub_cmd;
        ros::Timer publish_timer;

        //to control the movement of mast
        common::PID pid_joint_lift_seg_1;
        common::PID pid_joint_lift_seg_2;

        //Minimum/Maximum height of the mast, reported to C2
        float reported_min_height, reported_max_height, step_height;

        // Pointer to the update event connection
    private:
        event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ExtendableMastController)
}