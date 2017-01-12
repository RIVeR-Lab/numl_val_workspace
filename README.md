# NUML Val Workspace
This repo contains configuration files to quickly pull down NUML Valkyrie repositories using [vcstool](https://github.com/dirk-thomas/vcstool) for various workstations.

Use an existing .yaml file (or create a new .yaml file from the template) to pull the desired repositories with a command, such as:

`vcs import --input my-yaml.yaml ~/my_workspace/src/`

## Installing our codebase (example using full_workstation.yaml file)

```bash
cd ~
git clone git@github.com:RIVeR-Lab/numl_val_workspace.git
mkdir -p numl_catkin_ws/src
cd numl_catkin_ws/src
catkin config --init --mkdirs
catkin config --extend ~/ihmc_catkin_ws/install
vcs import < "$HOME"/numl_val_workspace/full_workstation.yaml
cd ..
catkin build
```
