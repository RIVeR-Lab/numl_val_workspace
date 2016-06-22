#!/bin/bash -i
  onlyroot="Do not run this as root."

if [ $(whoami) == 'root' ];then
    echo -e  $COLOR$onlyroot$MONO       #"Only root can do this operation."
    #DBG_MSG  "exit 5"
    exit 0
fi

cd ~

git clone git@github.com:RIVeR-Lab/numl_val_workspace.git

mkdir -p ~/numl_catkin_ws/src
cd ~/numl_catkin_ws/src
catkin_init_workspace
vcs import < "$HOME"/numl_val_workspace/full_workstation.yaml

cd ..
rosdep install --from-paths src -i -y
catkin_make

echo 'source ~/numl_catkin_ws/devel/setup.bash' >> ~/.bashrc
source ~/.bashrc
 
