#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include <sdf/sdf.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/msgs/laserscan_stamped.pb.h>
#include <gazebo/msgs/laserscan.pb.h>
#include <gazebo/common/common.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Subscriber.hh>
#include <gazebo/transport/Publisher.hh>

#include <surveillance_simulator/LrfCommand.h>
#include <surveillance_simulator/LrfStatus.h>

#include <ros/ros.h>

#include <string>
#include <sstream>

using namespace std;

//https://github.com/arpg/Gazebo/blob/master/gazebo/msgs/laserscan_stamped.proto
//
typedef const boost::shared_ptr<const gazebo::msgs::LaserScanStamped> StampedLaserScanPtr;


namespace gazebo
{
    class LrfController : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            // Store the pointer to the model
            this->model = _parent;
            this->sdf = _sdf;

            // Read parameters from sdf->Init(world)
            
            //Topic to which status messages are published to, composed as /modelname/<topic_name>
            std::string topic_name_pub_status;
            //Topic on which commands are recevied
            std::string topic_name_sub_commands;
            //Topic on which ray sensor publishes data
            std::string topic_name_ray_sensor;
            //Rate at which status is published
            int publish_rate;

            if(!readFromSdf("topic_name_pub_status", topic_name_pub_status))
            {
                return;
            }
            topic_name_pub_status = this->model->GetName() + "/" + topic_name_pub_status;

            if(!readFromSdf("topic_name_sub_commands", topic_name_sub_commands))
            {
                return;
            }
            topic_name_sub_commands = this->model->GetName() + "/" + topic_name_sub_commands;

            if(!readFromSdf("topic_name_ray_sensor", topic_name_ray_sensor))
            {
                return;
            }

            if(!readFromSdf("publish_rate", publish_rate))
            {
                return;
            }

            //Init ros related stuff
            if(!ros::isInitialized())
            {
                ROS_FATAL_STREAM("ROS plugin not initalized.");
                return;
            }

            //http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.Writing_the_Subscriber_Node
            // pub_state = node.advertise<surveillance_simulator::MastStatus>(topic_name_pub_status, 10);
            //publish_timer = node.createTimer(publish_rate, &ExtendableMastController::advertStatus, this);

            sub_lrf_cmds = node.subscribe(topic_name_sub_commands, 10, &LrfController::onLrfCmd, this);

            //http://gazebosim.org/tutorials?tut=custom_messages
            gznode = gazebo::transport::NodePtr(new gazebo::transport::Node());
            gznode->Init(_parent->GetName());
            this->sub_ray_sensor = gznode->Subscribe(
                topic_name_ray_sensor,
                &LrfController::onRaySensorMsg, 
                this
            );

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&LrfController::OnUpdate, this));

        }

        // Called by the world update start event
    public:


        void OnUpdate()
        {
        }

        // void onRaySensorMsg(const std::string& msg)
        void onRaySensorMsg(StampedLaserScanPtr& msg)
        {
            double dist = 0.0;
            if(msg->scan().ranges_size() > 0)
            {
                dist = msg->scan().ranges(0);
            }
            ROS_INFO("Got laser scan [%f]", dist);
        }

        void onLrfCmd(const surveillance_simulator::LrfCommand& msg)
        {
            ROS_INFO("Got LRF command [%d]", msg.command);
            
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

        // Pointer to the model
    private:
        //https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1Model.html
        physics::ModelPtr model;
        sdf::ElementPtr sdf;

        ros::NodeHandle node;
        ros::Publisher pub_state;
        ros::Subscriber sub_lrf_cmds;
        ros::Timer publish_timer;

        //For reading data from gazebo topics
        gazebo::transport::NodePtr gznode;
        gazebo::transport::SubscriberPtr sub_ray_sensor;

        // Pointer to the update event connection
    private:
        event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(LrfController)
}