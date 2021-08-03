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

#include <surveillance_simulator/MastStatus.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <string>
#include <sstream>

using namespace std;

namespace gazebo
{
    class EmStatePublisher : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            // Store the pointer to the model
            this->model = _parent;
            this->sdf = _sdf;

            // Read parameters from sdf

            if(!readFromSdf("topic_name", this->topic_name))
            {
                return;
            }
            this->topic_name = this->model->GetName() + "/" + this->topic_name;

            if(!readFromSdf("link_name_mast", this->link_name_mast))
            {
                return;
            }

            if(!readFromSdf("publish_rate", this->publish_rate))
            {
                return;
            }

            if(!readFromSdf("min_height", this->min_height))
            {
                return;
            }

            if(!readFromSdf("max_height", this->max_height))
            {
                return;
            }
            

            //Init ros related stuff
            if(!ros::isInitialized())
            {
                ROS_FATAL_STREAM("ROS plugin not initalized.");
                return;
            }

            //resolve reference to em_mast link for reading state in publish
            link_mast= model->GetLink(this->link_name_mast);
            if(link_mast == nullptr)
            {
                ROS_FATAL_STREAM("Unable to get reference to link em_mast.");
                return;
            }

            state_publisher = node.advertise<surveillance_simulator::MastStatus>(this->topic_name, 10);
            publish_timer = node.createTimer(this->publish_rate, &EmStatePublisher::advertStatus, this);

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&EmStatePublisher::OnUpdate, this));

        }

        // Called by the world update start event
    public:
        void OnUpdate()
        {
            // Apply a small linear velocity to the model.
            //this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
        }

        void advertStatus(const ros::TimerEvent& e)
        {
            //https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1Link.html
            //https://osrf-distributions.s3.amazonaws.com/ign-math/api/1.0.0/classignition_1_1math_1_1Pose3.html
            ignition::math::Pose3d pose = link_mast->WorldPose();
            ignition::math::Vector3 position = pose.Pos();
            
            surveillance_simulator::MastStatus status;
            status.min_height = this->min_height;
            status.max_height = this->max_height;
            status.current_height = position.Z();
            status.error_code = 0;

            state_publisher.publish(status);
            
        }

    private:
        template<typename T>
        inline bool readFromSdf(string element_name, T &store_to)
        {
            if(!this->sdf->HasElement(element_name))
            {
                ostringstream errMsg;
                errMsg << "Missing " << element_name << " in plugin sdf.";

                ROS_FATAL_STREAM(errMsg.str());
                return false;
            }
            store_to = this->sdf->Get<T>(element_name);

            return true;
        }

        // Pointer to the model
    private:
        //https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1Model.html
        physics::ModelPtr model;
        sdf::ElementPtr sdf;
        ros::NodeHandle node;
        ros::Publisher state_publisher;
        ros::Timer publish_timer;
        gazebo::physics::LinkPtr link_mast;

        //Topic to which status messages are published to, composed as /modelname/<topic_name>
        std::string topic_name;
        //The link which forms the moving part and is connected via prismatic joint to the base
        std::string link_name_mast;
        //Rate at which status is published
        int publish_rate;
        //Minimum/Maximum height of the mast, reported to C2
        float min_height, max_height;

        // Pointer to the update event connection
    private:
        event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(EmStatePublisher)
}