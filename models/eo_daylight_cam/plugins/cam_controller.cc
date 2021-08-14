#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include <sdf/sdf.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/rendering/Camera.hh>

#include <surveillance_simulator/Ptz.h>


#include <ros/ros.h>

#include <string>

using namespace std;

namespace gazebo
{
    class CameraController : public SensorPlugin
    {
    public:
        void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr _sdf)
        {
            if(!sensor)
            {
                gzerr << "Invalid sensor" << endl;
                return;
            }
            this->sdf = _sdf;

            _parentSensor = std::dynamic_pointer_cast<gazebo::sensors::CameraSensor>(sensor);
            if(!_parentSensor)
            {
                gzerr << "Expected sensor of type Camera" << endl;
                return;
            }

            _camera = _parentSensor->Camera();
            if(!_camera)
            {
                gzerr << "Expected a camera sensor" << endl;
                return;
            }

            _parentSensor->SetActive(true);


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
            topic_name_pub_status = getModelName() + "/" + topic_name_pub_status;

            if(!readFromSdf("topic_name_sub_commands", topic_name_sub_commands))
            {
                return;
            }
            topic_name_sub_commands = getModelName() + "/" + topic_name_sub_commands;            

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
            // pub_state = node.advertise<surveillance_simulator::LrfStatus>(topic_name_pub_status, 10);
            publish_timer = node.createTimer(publish_rate, &CameraController::advertStatus, this);

            sub_zoom_cmds = node.subscribe(topic_name_sub_commands, 10, &CameraController::onZoomCmd, this);
        }

        std::string getModelName()
        {
            string scopedName = this->_parentSensor->ParentName();
            int found_at = scopedName.find("::");
            if(found_at < 0){
                ROS_DEBUG("Unable to find scope in parent name");
            }else{
                return scopedName.substr(0, found_at);
            }

            return "";
        }

        // Called by the world update start event
    public:

        void onZoomCmd(const surveillance_simulator::Ptz& msg)
        {
            if(!_parentSensor->IsActive())
            {
                ROS_DEBUG("onZoomCmd - parent inactive.");
                return;
            }
            
            if(!_camera)
            {
                ROS_DEBUG("onZoomCmd - no camera reference.");
                return;
            }

            this->_zoomcmd = msg.zoomcmd;

            if(msg.zoomcmd == 0)
            {
                if(msg.hfov >= 0.05 && msg.hfov < 3.14)
                {
                    _camera->SetHFOV(msg.hfov);
                }
            }
        }

        void advertStatus(const ros::TimerEvent& e)
        {
            if(this->_zoomcmd == 2) //zoom in
            {
                if(this->_curr_zoom + 0.05 < 3.14)
                {
                    this->_curr_zoom += 0.05;
                    _camera->SetHFOV(this->_curr_zoom);
                }else{
                    this->_zoomcmd = 1; //stop zoom
                }
            }else if(this->_zoomcmd == 3) //zoom out
            {
                if(this->_curr_zoom - 0.05 >= 0.05)
                {
                    this->_curr_zoom -= 0.05;
                    _camera->SetHFOV(this->_curr_zoom);
                }else{
                    this->_zoomcmd = 1; //stop zoom
                }
            }
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
        gazebo::sensors::CameraSensorPtr _parentSensor{nullptr};
        gazebo::rendering::CameraPtr _camera{nullptr};
        sdf::ElementPtr sdf;

        ros::NodeHandle node;
        ros::Publisher pub_state;
        ros::Subscriber sub_zoom_cmds;
        ros::Timer publish_timer;
        
        uint _zoomcmd = 0;
        double _curr_zoom = 0.05;

        // Pointer to the update event connection
    private:
        event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_SENSOR_PLUGIN(CameraController)
}