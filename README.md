![ROS](https://upload.wikimedia.org/wikipedia/commons/b/bb/Ros_logo.svg)
# ROS Hacks Repo

## Overview
The repository is designed to make ROS developer's life easier.
After the installation usefull aliases and functions will be added to the terminal.
Everything was tested on Ubuntu 18.04.2 with ROS Melodic & ROS2 Dashing.

## Installation
Make sure that `tmux` is installed:

```shell
sudo apt install tmux
```


Clone the repo into some folder, for example:

```shell
git clone https://github.com/yossioo/ROS-Hacks.git ~/.ROS-Hacks
```

or if you like SSH😉:

```shell
git clone git@github.com:yossioo/ROS-Hacks.git ~/.ROS-Hacks
```

Navigate to the directory and execute the `setup.sh` script.

```shell
cd ~/.ROS-Hacks;  bash /setup.sh
```

The `~/.inputrc` file is saved to  `~/.inputrc.bak` prior to being overwritten.

Defaults for ROS versions are: Melodic for ROS1, and Dashing for ROS2. Adjust in `~/aliases.sh` if needed.

## Terminal shortcuts
The shortcuts can be run in a **new** terminal after the installation. _Note: re-sourcing `~/.bashrc` isn't enough._

Complete list of the shortcuts can be seen in  [`inputrc`](inputrc) file.
Few of them are presented below:

| Shortcut | Executed command | Description |
| ------ | ------ |  ------ |
| F3 | `select_ws` | Displays ROS workspace selection dialog. |
| Shift-F3 | `prompt_new_ws` | Displays new ROS workspace creation dialog. |
| F3 | `select_ws` | Displays workspace selection dialog. |
| F9 | `rebuild_curr_ws` | Re-builds the currently selected workspace and sources it. |
| Shift-F12 | `set-quick-command` | Saves currently typed-in command for quick-launch<sup>1</sup>. |
| F12 | `exec-quick-command` | Executes the saved command in detached tmux session. |
| ... | .... |....... |

<sup>1</sup> The function is primarily targeted for quick launching and killing of Gazebo worlds using various `roslaunch` commands. A quick execution commands are saved for each workspace separately.

Example of usage of Quick Command:
Type in terminal `roslaunch gazebo_ros empty_world.launch`, hit **`Shift-F12`**. The command will be saved for currently sourced workspace. Hit **`F12`** to execute the command in background tmux session. The Gazebo client GUI will show up. Hit  **`Ctrl-F12`** to stop the session, killing Gazebo along with all ROS nodes (currently only on ROS1).

## Aliases
Complete list of the aliases can be seen in  [`aliases.sh`](aliases.sh) file.
Few of them are presented below:

| Alias | Expanded command | Description |
| ------ | ------ |  ------ |
| `sc` | `source_ws $(cat $WS_FILE)` | Sources the currently selected workspace. |
| `o` | `sudo chown -R $USER:$USER` | Allows easy ownership change for selected files. |
| `x` | `chmod +x ` | Adds execution permissions for desired files. |
| ... | .... |....... |

## Functions
Complete list of the functions can be seen in  [`functions.sh`](functions.sh) file.

The function names are self explanatory, but should a need arise, some documentation can be added.

## Removal
Clear `~/.bashrc` file from the added lines. Delete the repo. Voilà!
