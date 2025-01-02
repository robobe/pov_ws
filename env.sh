source install/setup.bash
export PS1="🐳  \[\033[1;36m\]\h \[\033[1;34m\]\W\[\033[0;35m\] \[\033[1;36m\]# \[\033[0m\]"
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:`pwd`/src/pov_gazebo/worlds:`pwd`/src/pov_description/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/build/pov_gazebo
source alias.sh