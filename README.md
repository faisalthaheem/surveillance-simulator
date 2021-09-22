# STANAG 4586 compliant surveillance simulator

This project aims to develop a fully functional mission module that can be mounted on a virtual unmanned ground vehicle for simulating ISR missions. The module mimicks functionalities of a real life mission module and supports a variety of sensors, actuators and features which can be remotely controlled by a command and control application. Sensors/actuators currently available include
- Extendable mast
- Electro Optic camera with RTSP feed of the simulated virtual environment
- Laser Range Finder (LRF)

Checkout the demo video at youtube by clicking on the following image
[![Demo preview on Youtube](https://img.youtube.com/vi/U5bGR7-89vI/0.jpg)](https://www.youtube.com/watch?v=U5bGR7-89vI)

## Building and Running

> Ensure following exists on your environment, this project was developed using Ubuntu 20.04 LTS

Dependency | Notes
--- | ---
Ros Version|Noetic (http://wiki.ros.org/noetic/Installation/Ubuntu)
Gazebo | http://gazebosim.org/tutorials?tut=install_ubuntu
GStreamer | sudo apt install gstreamer1.0-plugins-base gstreamer1.0-plugins-base-apps gstreamer1.0-plugins-good gstreamer1.0-tools gstreamer1.0-plugins-bad gstreamer1.0-plugins-rtp gstreamer1.0-plugins-ugly gstreamer1.0-rtsp gstreamer1.0-qt5 libgstrtspserver-1.0-0 libgstrtspserver-1.0-dev gstreamer1.0-libav



Assuming you already have ros and gazebo setup, begin by cloning this repository into a new directory

> Setup workspace
```bash
mkdir -p surv/src && cd surv
catkin_make
source devel/setup.bash
```

> Install dependencies
```bash
pip3 install stanag4586edav1
pip3 install stanag4586vsm
```

> Checkout project and build
```bash
cd src
git clone https://github.com/faisalthaheem/surveillance-simulator
pushd
catkin_make
popd
./run-gazebo.sh
```

On first run it will take some time for the models to be downloaded from internet, subsequent runs should be faster.

## Configuring

To configure the RTSP streaming, you must set the machine's reachable ip address in the file launch/gazebo.launch - look for the following code block and update

```xml
<env name="EXTERNAL_RTSP_IP_ADDRESS" value="10.10.20.102" />
```
