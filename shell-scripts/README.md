# About
Scripts that make it possible to run tools in virtualized environments.
Gazebo can be launched in lxd container with the help of provided scripts with hardware acceleration enabled and this project directory mounted under /home/ubuntu/work/simulator

Special thanks to people behind the information on the following websites
- https://blog.simos.info/running-x11-software-in-lxd-containers/
- https://www.cyberciti.biz/faq/how-to-add-or-mount-directory-in-lxd-linux-container/
- https://ubuntu.com/blog/custom-user-mappings-in-lxd-containers

Following sequence should be followed to launch a container running gazebo
Needed to be run once only after installing lxd
- snap-lxd-track-latest.sh
- lxd-create-x11-profile.sh
- lxd-fs-id-map.sh

Then onwards whenever a container is required
- gazebo-create-container.sh
- gazebo-install-to-container.sh (installs ros 2 noetic and gazebo 9)

Finally to login to conainer
- gazebo-login-to-container.sh