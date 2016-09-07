#!/bin/bash -i
  onlyroot="Do not run this as root."

if [ $(whoami) == 'root' ];then
    echo -e  $COLOR$onlyroot$MONO       #"Only root can do this operation."
    #DBG_MSG  "exit 5"
    exit 0
fi

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

sudo apt-get update
sudo apt-get -y upgrade

sudo groupadd ros
sudo groupadd pgrimaging
sudo usermod -a -G ros $USER
sudo usermod -a -G dialout $USER
sudo usermod -a -G pgrimaging $USER

# 2 new dependicies: libdrm-nouveau2 libdrm-radeon1  / Shoul we add them ?
sudo apt-get -y install  libglew-dev libcheese7 libcheese-gtk23 libclutter-gst-2.0-0 libcogl15 libclutter-gtk-1.0-0 libclutter-1.0-0  xserver-xorg-input-all
sudo apt-get -y install binutils ca-certificates cpp cpp-4.8 curl fontconfig fontconfig-config fonts-dejavu-core g++ g++-4.8 gcc gcc-4.8 git git-flow htop iso-codes krb5-locales libamd2.3.1 libasan0 libatomic1 libaudio2 libavahi-client3 libavahi-common-data libavahi-common3 libblas3 libc-dev-bin libc6-dev libcamd2.3.1 libccolamd2.8.0 libcholmod2.1.2 libcloog-isl4 libcolamd2.8.0 libcups2 libdrm-intel1 libelf1 libexpat1-dev libfontconfig1 libfreetype6 libgcc-4.8-dev libgfortran3 libgl1-mesa-dri libgl1-mesa-glx libglapi-mesa libglib2.0-0 libglib2.0-data libgmp10 libgomp1 libgssapi-krb5-2 libgstreamer-plugins-base1.0-0 libgstreamer1.0-0 libice6 libisl10 libitm1 libjbig0 libjpeg-turbo8 libjpeg8 libk5crypto3 libkeyutils1 libkrb5-3 libkrb5support0 liblapack3 liblcms2-2 libllvm3.4 libmpc3 libmpfr4 libmysqlclient18 liborc-0.4-0 libpciaccess0 libpython-dev libpython-stdlib libpython2.7 libpython2.7-dev libpython2.7-minimal libpython2.7-stdlib libqt4-dbus libqt4-declarative libqt4-designer libqt4-help libqt4-network libqt4-opengl libqt4-script libqt4-scripttools libqt4-sql libqt4-sql-mysql libqt4-svg libqt4-test libqt4-xml libqt4-xmlpatterns libqtassistantclient4 libqtcore4 libqtdbus4 libqtgui4 libqtwebkit4 libquadmath0 libsm6 libstdc++-4.8-dev libtiff5 libtsan0 libtxc-dxtn-s2tc0 libumfpack5.6.2 libwebp5 libwebpmux1 libx11-6 libx11-data libx11-xcb1 libxau6 libxcb-dri2-0 libxcb-dri3-0 libxcb-glx0 libxcb-present0 libxcb-sync1 libxcb1 libxdamage1 libxdmcp6 libxext6 libxfixes3 libxi6 libxml2 libxrender1 libxshmfence1 libxslt1.1 libxt6 libxxf86vm1 linux-libc-dev lm-sensors manpages manpages-dev mysql-common nano openssl python python-decorator python-dev python-imaging python-minimal python-numpy python-pil python-pip python-qt4 python-rosdep python-scipy python-sip python-six python-support python-vcstool python2.7 python2.7-dev python2.7-minimal qdbus qtchooser qtcore4-l10n ros-indigo-catkin sgml-base shared-mime-info syslog-ng-core vim wget x11-common xml-core nfs-common libffi-dev


sudo apt-get -y install -f

sudo apt-get -y install ros-indigo-ros-base
sudo apt-get -y install syslog-ng

cd ~/Downloads

# old config file (only config file was updated)
# wget https://dl.dropboxusercontent.com/u/71518/nasa-val-system-config_3.0.0.0-2016.05.21.15.36.27-39e7fb83_amd64.deb
wget https://dl.dropboxusercontent.com/u/92698211/nasa-val-system-config_3.0.3.0-2016.07.25.00.36.32-e9f2234d_amd64.deb
wget https://dl.dropboxusercontent.com/u/71518/nasa-indigo-workspace_1.0.0_amd64.deb
wget https://dl.dropboxusercontent.com/u/71518/python-pyqtgraph_0.9.8-1_all.deb

sudo dpkg -i nasa-indigo-workspace*.deb nasa-val-system-config*.deb python-pyqtgraph*.deb

cd -

sudo rosdep init

# Old yamls files from github
# sudo sh -c 'echo "yaml https://raw.githubusercontent.com/NASA-JSC-Robotics/nasa_common_rosdep/master/nasa-common.yaml"  > /etc/ros/rosdep/sources.list.d/10-nasa-common.list'
# sudo sh -c 'echo "yaml https://raw.githubusercontent.com/NASA-JSC-Robotics/nasa_common_rosdep/master/nasa-trusty.yaml"  > /etc/ros/rosdep/sources.list.d/11-nasa-trusty.list'
# sudo sh -c 'echo "yaml https://raw.githubusercontent.com/NASA-JSC-Robotics/nasa_common_rosdep/master/nasa-indigo-gazebo7.yaml" > /etc/ros/rosdep/sources.list.d/12-nasa-gazebo7.list'


sudo sh -c 'echo "yaml https://gitlab.com/nasa-jsc-robotics/nasa_common_rosdep/raw/master/nasa-common.yaml"  > /etc/ros/rosdep/sources.list.d/10-nasa-common.list'
sudo sh -c 'echo "yaml https://gitlab.com/nasa-jsc-robotics/nasa_common_rosdep/raw/master/nasa-trusty.yaml"  > /etc/ros/rosdep/sources.list.d/11-nasa-trusty.list'
sudo sh -c 'echo "yaml https://gitlab.com/nasa-jsc-robotics/nasa_common_rosdep/raw/master/nasa-indigo-gazebo7.yaml" > /etc/ros/rosdep/sources.list.d/12-nasa-indigo-gazebo7.list'


rosdep update

python /usr/local/share/nasa/setup_nasa_val_user_bash.py

cat <<EOT >> ~/.bashrc
source /opt/ros/indigo/setup.bash
if [ -f ~/.bash_nasa_val ]; then
    source ~/.bash_nasa_val
fi
EOT

source ~/.bashrc

mkdir -p ~/val_indigo/src && cd ~/val_indigo/src
catkin_init_workspace

# old val_workspaces repo
# git clone -b develop git@github.com:NASA-JSC-Robotics/val_workspaces.git ~/val_workspaces

# in order to run this command user needs create ssh key and copy-paste it to his/her gitlab account.
git clone -b develop git@gitlab.com:nasa-jsc-robotics/val_workspaces.git ~/val_workspaces

vcs import --input ~/val_workspaces/public_developer_workspace.yaml ~/val_indigo/src/
rm -rf ~/val_indigo/src/val_vision

source ~/.bashrc

cd ~/val_indigo
vcs custom --args checkout master
rosdep install --from-paths src -i -y
rosdep install --from-paths src -i -y
rosdep install --from-paths src -i -y

catkin_make install
source ~/.bashrc
