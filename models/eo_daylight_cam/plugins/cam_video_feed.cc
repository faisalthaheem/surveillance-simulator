#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <sdf/sdf.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/rendering/Camera.hh>

#include <gst/gst.h>

#include <string>

using namespace std;

namespace gazebo
{
    static void* start_gst_thread(void* plugin) 
    {
        CameraVideoFeed* plugin = (CameraVideoFeed*)plugin;
        plugin->gstreamer_main_loop();
        return nullptr;
    }
    
    class CameraVideoFeed : public SensorPlugin
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
                gzerr << "Expected sensor of type Camera";
                return;startGstThread
            }

            _camera = _parentSensor->Camera();
            if(!_camera)
            {
                gzerr << "Expected a camera sensor";
                return;
            }

            _parentSensor->SetActive(true);


            // Read parameters from sdf->Init(world)

            if(!readFromSdf("gst_pipeline", _gst_pipeline))
            {
                gzerr << "GST pipeline definition missing";
                return;
            }
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
        void gstreamer_main_loop()
        {
            gst_init(nullptr, nullptr);

            this->_gst_main_loop = g_main_loop_new(nullptr, FALSE);
            if (!this->_gst_main_loop) {
                gzerr << "Unable to create gst main loop." << endl;
                return;
            }
    
            unique_ptr<GError*> err;
            unique_ptr<GstElement*> pipeline = gst_parse_launch(this->_gst_pipeline, &err);
            if (!pipeline) {
                gzerr << "Errorneous pipeline." << endl;
                return;
            }

            // this->_pip

            // pipeline->
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
        
        private: std::string _gst_pipeline;
        private: GstPipeline
        private: pthread_t _gst_thread;

        // Pointer to the update event connection
    private:
        event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_SENSOR_PLUGIN(CameraVideoFeed)
}