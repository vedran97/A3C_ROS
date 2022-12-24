#! /bin/bash
set -e

SCRIPT_LOCATION=$(dirname $0)
SCRIPT_NAME=$(basename $0)

USERNAME=$1
WORKSPACE_PATH=$2

export DEBIAN_FRONTEND=noninteractive

apt-get update

apt-get -y install --no-install-recommends \
    $(echo $(cat $SCRIPT_LOCATION/../dev-tools/common.txt))

apt-get update

apt-get -y install --no-install-recommends \
    $(echo $(cat $SCRIPT_LOCATION/../dev-tools/ros-packages.txt))

apt-get upgrade -y

# Source ROS
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc

# Source workspace
echo "source ${WORKSPACE_PATH}/devel/setup.bash" >> /home/$USERNAME/.bashrc

# Define Python version for ros
echo "export ROS_PYTHON_VERSION=3" >> /home/$USERNAME/.bashrc


source /home/$USERNAME/.bashrc

pip3 install --upgrade \
    $(echo $(cat $SCRIPT_LOCATION/../dev-tools/python3-deps.txt))

rosdep update
sudo apt update
rosdep install --from-paths $SCRIPT_LOCATION/../src --ignore-src -r -y

