#!/bin/bash

wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | sudo apt-key add - 
sudo sh -c 'echo "deb [arch=amd64] http://dl.google.com/linux/chrome/deb/ stable main" >> /etc/apt/sources.list.d/google.list'
sudo apt-get update
sudo apt-get -y install google-chrome-stable

sudo apt-get -y install aptitude
sudo apt-get -y install terminator
sudo apt-get -y install qtcreator
sudo apt-get -y install stress
sudo apt-get -y install fping
sudo apt-get -y install iftop 
sudo apt-get -y install lm-sensors 
sudo apt-get -y install vim 
sudo apt-get -y install emacs 
sudo apt-get -y install htop 
sudo apt-get -y install openssh-server
sudo apt-get -y install gimp
sudo apt-get -y install gitg
sudo apt-get -y install gitk
sudo apt-get -y install screen
sudo apt-get -y install tshark
sudo apt-get -y install wireshark
sudo apt-get -y install ntp

cd ~/Downloads/
wget http://mirror.cc.columbia.edu/pub/software/eclipse/technology/epp/downloads/release/mars/2/eclipse-java-mars-2-linux-gtk-x86_64.tar.gz
tar xvf eclipse-java-mars-2-linux-gtk-x86_64.tar.gz
sudo mv eclipse /opt/
echo '[Desktop Entry]' >> eclipse.desktop
echo 'Name=Eclipse' >> eclipse.desktop
echo 'Type=Application' >> eclipse.desktop
echo 'Exec=/opt/eclipse/eclipse' >> eclipse.desktop
echo 'Terminal=false' >> eclipse.desktop
echo 'Icon=/opt/eclipse/icon.xpm' >> eclipse.desktop
echo 'Comment=Integrated Development Environment' >> eclipse.desktop
echo 'NoDisplay=false' >> eclipse.desktop
echo 'Categories=Development;IDE' >> eclipse.desktop
echo 'Name[en]=Eclipse' >> eclipse.desktop
sudo mv eclipse.desktop /usr/share/applications/eclipse.desktop
cd /usr/local/bin
sudo ln -s /opt/eclipse/eclipse