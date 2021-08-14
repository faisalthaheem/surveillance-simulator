#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>


#include <gazebo/msgs/msgs.hh>
#include <sdf/sdf.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Subscriber.hh>
#include <gazebo/transport/Publisher.hh>

#include <surveillance_simulator/Ptz.h>         
#include <surveillance_simulator/RelativePanTilt.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <string>
#include <sstream>

using namespace std;

typedef std::map< std::string, gazebo::physics::JointPtr > ModelJoints;

namespace gazebo
{
    class SensorMountController : public ModelPlugin
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
            std::string topic_name_sub_ptz_targets;
            std::string topic_name_sub_relative_ptz;
            //The link which forms the base and is panned
            std::string link_name_pan;
            //The name of joint that connects this mount to the parent
            std::string joint_name_pan;
            //The link which forms the mount and is tilted
            std::string link_name_tilt;
            //The name of joint that connects the payload to this mount
            std::string joint_name_tilt;
            //Rate at which status is published
            int publish_rate;

            if(!readFromSdf("topic_name_sub_ptz_targets", topic_name_sub_ptz_targets))
            {
                return;
            }
            topic_name_sub_ptz_targets = this->model->GetName() + "/" + topic_name_sub_ptz_targets;

            if(!readFromSdf("topic_name_sub_relative_ptz", topic_name_sub_relative_ptz))
            {
                return;
            }
            topic_name_sub_relative_ptz = this->model->GetName() + "/" + topic_name_sub_relative_ptz;

            if(!readFromSdf("topic_name_pub_status", topic_name_pub_status))
            {
                return;
            }
            topic_name_pub_status = this->model->GetName() + "/" + topic_name_pub_status;

            if(!readFromSdf("link_name_pan", link_name_pan))
            {
                return;
            }

            if(!readFromSdf("joint_name_pan", joint_name_pan))
            {
                return;
            }
            // joint_name_pan = this->model->GetName() + "::" + joint_name_pan;

            if(!readFromSdf("link_name_tilt", link_name_tilt))
            {
                return;
            }

            if(!readFromSdf("joint_name_tilt", joint_name_tilt))
            {
                return;
            }
            // joint_name_pan = this->model->GetName() + "::" + joint_name_pan;

            if(!readFromSdf("publish_rate", publish_rate))
            {
                return;
            }

            if(!readFromSdf("pan_rate", this->pan_rate))
            {
                return;
            }

            if(!readFromSdf("tilt_rate", this->tilt_rate))
            {
                return;
            }

            //Init ros related stuff
            if(!ros::isInitialized())
            {
                ROS_FATAL_STREAM("ROS plugin not initalized.");
                return;
            }

            /*******************************************************************/
            /*** resolve reference to pan link for reading state in publish ***/
            /*******************************************************************/
            link_pan= model->GetLink(link_name_pan);
            if(link_pan == nullptr)
            {
                ROS_FATAL("Unable to get reference to link [%s].", link_name_pan.c_str());
                return;
            }

            //https://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Joint.html
            //https://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/namespacegazebo_1_1physics.html#af7e4c06bce794eef119784b85ce330f5
            //https://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1JointController.html

            ModelJoints available_joints = model->GetJointController()->GetJoints();
            
            this->joint_pan = model->GetJoint(joint_name_pan);
            if(!this->joint_pan)
            {
                ROS_FATAL("Unable to find joint [%s] as specified in sdf.", joint_name_pan.c_str());
                printAvailableJoints(available_joints);
                return;
            }


            /*******************************************************************/
            /*** resolve reference to tilt link for reading state in publish ***/
            /*******************************************************************/
            link_tilt= model->GetLink(link_name_tilt);
            if(link_tilt == nullptr)
            {
                ROS_FATAL("Unable to get reference to link [%s].", link_name_tilt.c_str());
                return;
            }
            this->joint_tilt = model->GetJoint(joint_name_tilt);
            if(!this->joint_tilt)
            {
                ROS_FATAL("Unable to find joint [%s] as specified in sdf.", joint_name_tilt.c_str());
                printAvailableJoints(available_joints);
                return;
            }

            //get limits
            this->limit_tilt_lower = this->joint_tilt->LowerLimit();
            this->limit_tilt_upper = this->joint_tilt->UpperLimit();

            /*******************************************************************/
            /*** Setup PID controllers ***/
            /*******************************************************************/

            this->pid_pan = common::PID(20.0, 50.0, 10.0);
            this->pid_tilt = common::PID(20.0, 50.0, 10.0);

            this->model->GetJointController()->SetPositionPID(
                this->joint_pan->GetScopedName(), this->pid_pan);

            this->model->GetJointController()->SetPositionPID(
                this->joint_tilt->GetScopedName(), this->pid_tilt);

            //Set initial target
            this->model->GetJointController()->SetPositionTarget(
                this->joint_pan->GetScopedName(), this->target_pan);

            this->model->GetJointController()->SetPositionTarget(
                this->joint_tilt->GetScopedName(), this->target_tilt);

            //http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.Writing_the_Subscriber_Node
            // pub_state = node.advertise<surveillance_simulator::MastStatus>(topic_name_pub_status, 10);
            //publish_timer = node.createTimer(publish_rate, &ExtendableMastController::advertStatus, this);

            sub_ptz_targets = node.subscribe(topic_name_sub_ptz_targets, 10, &SensorMountController::onPtzTargets, this);
            sub_relative_ptz = node.subscribe(topic_name_sub_relative_ptz, 10, &SensorMountController::onRelativePtzTargets, this);


            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&SensorMountController::OnUpdate, this));

        }

        // Called by the world update start event
    public:
        void OnUpdate()
        {
            //https://answers.gazebosim.org//question/18816/how-correctly-move-joints-by-setpositionpid-and-setpositiontarget/
            this->model->GetJointController()->Update();
        }

        void onRelativePtzTargets(const surveillance_simulator::RelativePanTilt& msg)
        {
            if(msg.pan_direction < 0)
            {
                this->target_pan += this->pan_rate;

            }else if(msg.pan_direction > 0)
            {
                this->target_pan -= this->pan_rate;
            }

            if(msg.tilt_direction > 0 && (this->target_tilt + this->tilt_rate) <= this->limit_tilt_upper)
            {
                this->target_tilt += this->tilt_rate;
            
            }else if(msg.tilt_direction < 0 && (this->target_tilt - this->tilt_rate) >= this->limit_tilt_lower)
            {
                this->target_tilt -= this->tilt_rate;
            }

            this->model->GetJointController()->SetPositionTarget(
                this->joint_pan->GetScopedName(), this->target_pan);

            this->model->GetJointController()->SetPositionTarget(
                this->joint_tilt->GetScopedName(), this->target_tilt);
        }

        void onPtzTargets(const surveillance_simulator::Ptz& msg)
        {
            if(msg.pan != -1000.0f)
            {
                this->target_pan = msg.pan;
                this->model->GetJointController()->SetPositionTarget(
                    this->joint_pan->GetScopedName(), this->target_pan);
            }
            
            if(msg.tilt != -1000.0f && msg.tilt >= this->limit_tilt_lower && msg.tilt <= this->limit_tilt_upper)
            {
                this->target_tilt = msg.tilt;
                this->model->GetJointController()->SetPositionTarget(
                    this->joint_tilt->GetScopedName(), this->target_tilt);
            }
        }

        void advertStatus(const ros::TimerEvent& e)
        {
             
            // surveillance_simulator::MastStatus status;
            // status.min_height = this->min_height;
            // status.max_height = this->max_height;
            // status.current_height = getMastHeight();
            // status.error_code = 0;

            // pub_state.publish(status);
            
        }


    private:
        template<typename T>
        inline bool readFromSdf(string element_name, T &store_to)
        {
            if(!this->sdf->HasElement(element_name))
            {
                // ostringstream errMsg;
                // errMsg << "Missing " << element_name << " in plugin sdf.";
                // ROS_FATAL_STREAM(errMsg.str());

                ROS_FATAL("Missing [%s] element in sdf.", element_name.c_str());

                return false;
            }
            store_to = this->sdf->Get<T>(element_name);

            return true;
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

        // Pointer to the model
    private:
        //https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1Model.html
        physics::ModelPtr model;

        gazebo::physics::JointPtr joint_pan; 
        gazebo::physics::JointPtr joint_tilt; 
        gazebo::physics::LinkPtr link_pan;
        gazebo::physics::LinkPtr link_tilt;

        sdf::ElementPtr sdf;

        ros::NodeHandle node;
        ros::Publisher pub_state;
        ros::Subscriber sub_ptz_targets;
        ros::Subscriber sub_relative_ptz;
        ros::Timer publish_timer;

        //to control the pan
        common::PID pid_pan;
        //to control the tilt
        common::PID pid_tilt;

        //Minimum/Maximum height of the mast, reported to C2
        float pan_rate = 0.1f, tilt_rate = 0.1f, target_pan = 0.0f, target_tilt =0.0f;

        float limit_tilt_lower = -1.57f;
        float limit_tilt_upper = 1.57f;

        // Pointer to the update event connection
    private:
        event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(SensorMountController)
}