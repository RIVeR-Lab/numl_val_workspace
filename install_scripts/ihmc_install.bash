#!/bin/bash
  onlyroot="Do not run this as root."

if [ $(whoami) == 'root' ];then
    echo -e  $COLOR$onlyroot$MONO       #"Only root can do this operation."
    #DBG_MSG  "exit 5"
    exit 0
fi

sudo add-apt-repository ppa:webupd8team/java -y
sudo apt-get update
echo oracle-java8-installer shared/accepted-oracle-license-v1-1 select true | sudo /usr/bin/debconf-set-selections
sudo apt-get install -y oracle-java8-installer
echo 'export JAVA_HOME=/usr/lib/jvm/java-8-oracle' >> ~/.bashrc
source ~/.bashrc

cd ~

git clone https://github.com/ihmcrobotics/ihmc_workspaces.git

mkdir -p ~/ihmc_catkin_ws/src
cd ~/ihmc_catkin_ws/src
catkin_init_workspace
vcs import < "$HOME"/ihmc_workspaces/catkin_workspaces/ihmc_valkyrie_developer_workspace.yaml

cd ..
rosdep install --from-paths src -i -y
catkin_make install

echo 'source ~/ihmc_catkin_ws/install/setup.sh' >> ~/.bashrc
source ~/.bashrc
 
