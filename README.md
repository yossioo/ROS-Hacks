![ROS](https://upload.wikimedia.org/wikipedia/commons/b/bb/Ros_logo.svg)
# ROS Hacks Repo

## Overview
The repository presents several files designed to make ROS developer's life easier.
After the installation a few usefull aliases and functions added to the terminal.
Everything was tested on Ubuntu 18.04.2 with ROS Melodic & ROS2 Dashing.

## Installation
Make sure that `tmux` is installed:

```sudo apt install tmux```


Clone the repo into some folder, for example:

```git clone https://github.com/yossioo/ROS-Hacks.git ~/.ROS-Hacks```

or if you like SSH😉 and like to keep things:

```git clone git@github.com:yossioo/ROS-Hacks.git```

Navigate to the directory and execute the `setup.sh` script (make sure it has execute permissions).

```cd ~/.ROS-Hacks; ./setup.sh```

The `~/.inputrc` file is saved to  `~/.inputrc.bak` prior to being overwritten.

Defaults for ROS versions are: Melodic for ROS1, and Dashing for ROS2. Adjust in ``~/aliases.sh` if needed.

## Terminal shortcuts
The shortcuts can be run immediately after the installation in new terminal. _Note: re-sourcing `~/.bashrc` isn't enough._

Complete list of the shortcuts can be seen in  [`inputrc`](inputrc) file.
Few of them presented below:

| Shortcut | Executed command | Description |
| ------ | ------ |  ------ |
| F3 | select_ws | Display ROS workspace selection dialog. |
| Shift-F3 | prompt_new_ws | Display new ROS workspace creation dialog. |
| F3 | select_ws | Display workspace selection dialog. |
| F9 | rebuild_curr_ws | Re-builds the currently selected workspace and sources it. |
| Shift-F12 | set-quick-command | Saves currently typed in command for quick-launch<sup>1</sup>. |
| F12 | exec-quick-command | Executes the saved command in detached tmux session. |
| ... | .... |....... |

<sup>1</sup> The function is primarily targeted for quick launching and killing of Gazebo worlds using various `roslaunch` commands. A quick execution command is saved for each workspace separately.

## Aliases
Complete list of the aliases can be seen in  [`aliases.sh`](aliases.sh) file.
Few of them presented below:

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
