#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <sdf/sdf.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/rendering/Camera.hh>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

#include <string>

using namespace std;

namespace gazebo
{
    
    //forward decl
    static void* run_gst_loop(void* plugin);
    
    class CameraVideoFeed : public SensorPlugin
    {
    public:

        virtual ~CameraVideoFeed()
        {
            stopGst();
        }


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
                return;
            }

            _camera = _parentSensor->Camera();
            if(!_camera)
            {
                gzerr << "Expected a camera sensor";
                return;
            }

            _parentSensor->SetActive(true);


            // Read parameters from sdf->Init(world)

            if(!readFromSdf("gst_pipeline", _pipeline_to_launch))
            {
                gzerr << "GST pipeline definition missing";
                return;
            }

            if(!readFromSdf("image_width", this->_image_width))
            {
                return;
            }

            if(!readFromSdf("image_height", this->_image_height))
            {
                return;
            }
            

            startGst();
        }

        void startGst()
        {
            //start gstreamer
            pthread_create(&this->_gst_thread, NULL, run_gst_loop, this);

            //begin receiving camera frames
            if(!this->camFrameConnection)
            {
                this->camFrameConnection = this->_camera->ConnectNewImageFrame(
                    boost::bind(&CameraVideoFeed::OnNewFrame, this, _1));
            }
        }

        void stopGst()
        {
            if(this->camFrameConnection)
            {
                this->camFrameConnection->~Connection();
            }

            if(this->_gst_loop) 
            {
                g_main_loop_quit(this->_gst_loop);
            }

            pthread_join(this->_gst_thread, nullptr);
        }

    public:
        void OnNewFrame(const unsigned char *image_buffer)
        {
            if(!this->_pipeline_ready)
            {
                return;
            }

            unsigned int size = this->_image_width * this->_image_width * 3;
            GstBuffer* gst_buffer = gst_buffer_new_allocate(NULL, size, NULL);

            if (!gst_buffer)
            {
                gzerr << "Unable to allocate gst buffer";
                return;
            }

            GstMapInfo gstmap;

            if (!gst_buffer_map(gst_buffer, &gstmap, GST_MAP_WRITE)) 
            {
                gzerr << "Unable to map buffer to gst map";
                return;
            }

            // Color Conversion from RGB to YUV
            memcpy(gstmap.data, image_buffer, size);
            gst_buffer_unmap(gst_buffer, &gstmap);
        
            GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(this->_gst_app_src), gst_buffer);

            if (ret != GST_FLOW_OK)
            {
                gzerr << "Unable to push buffer to app src";
                g_main_loop_quit(this->_gst_loop);
            }
        }

        void gstreamer_main_loop()
        {
            gst_init(nullptr, nullptr);

            this->_gst_loop = g_main_loop_new(nullptr, FALSE);
            if (!this->_gst_loop) {
                gzerr << "Unable to create gst main loop." << endl;
                return;
            }
    
            GError* err;
            GstElement *pipeline_element = gst_parse_launch(this->_pipeline_to_launch.c_str(), &err);
            if (!pipeline_element) {
                gzerr << "Errorneous pipeline." << endl;
                return;
            }

            this->_gst_pipeline = GST_PIPELINE(pipeline_element);
            this->_gst_app_src = gst_bin_get_by_name(&this->_gst_pipeline->bin, "appsrc");
            gst_object_ref(this->_gst_app_src);
            
            gst_element_set_state(pipeline_element, GST_STATE_PLAYING);
            _pipeline_ready = true;
            g_main_loop_run(this->_gst_loop);

            gst_element_set_state(pipeline_element, GST_STATE_NULL);
            gst_object_unref(GST_OBJECT(pipeline_element));
            gst_object_unref(GST_OBJECT(this->_gst_app_src));
            g_main_loop_unref(this->_gst_loop);

            this->_gst_app_src = nullptr;
            this->_gst_loop = nullptr;

        }

    private:
        template<typename T>
        inline bool readFromSdf(string element_name, T &store_to)
        {
            if(!this->sdf->HasElement(element_name))
            {

                gzerr << "Missing " << element_name.c_str() << "element in sdf.";

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

        private: std::string _pipeline_to_launch;
        private: unsigned int _image_width;
        private: unsigned int _image_height;
        private: GMainLoop* _gst_loop;
        private: GstPipeline* _gst_pipeline;
        private: GstElement* _gst_app_src;
        private: pthread_t _gst_thread;
        private: bool _pipeline_ready = false;

        // Pointer to the update event connection
    private:
        event::ConnectionPtr camFrameConnection;
    };

    static void* run_gst_loop(void* plugin) 
    {
        CameraVideoFeed* cam = (CameraVideoFeed*)plugin;
        cam->gstreamer_main_loop();
        return nullptr;
    }

    // Register this plugin with the simulator
    GZ_REGISTER_SENSOR_PLUGIN(CameraVideoFeed)
}