#!/bin/bash
  onlyroot="Do not run this as root."
  
if [ $(whoami) == 'root' ];then
    echo -e  $COLOR$onlyroot$MONO       #"Only root can do this operation."
    #DBG_MSG  "exit 5"
    exit 0
fi

cd ~/numl_val_workspace/install_scripts

sudo chmod +x useful_software_install.bash
sudo chmod +x nasa_install.bash
sudo chmod +x ihmc_install.bash
sudo chmod +x numl_install.bash

./useful_software_install.bash
./nasa_install.bash
./ihmc_install.bash
./numl_install.bash