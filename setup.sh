#!/bin/bash

source aliases.sh
printf "${LIGHT_BLUE_TXT}Installing ROS-Hacks${NC}.\n"

# Add to ~/.bashrc:
echo "## ROS-HACKS entries ##" >>~/.bashrc
echo "PS1=' \[\e[1;32m\]\u\[\033[00m\] \[\e[32m\]\$(get_current_ws_name):\$ROS_DOMAIN_ID\[\033[00m\] \[\033[03;94m\]\w\[\033[00m\]\[\033[38;5;51m\]\$(__git_ps1)\[\033[00m\]:\n\$ '"  >>~/.bashrc
echo "source ${PWD}/aliases.sh" >>~/.bashrc
echo "source ${PWD}/functions.sh" >>~/.bashrc
echo "get_current_ws" >>~/.bashrc
echo 'source_ws $curr_ws' >>~/.bashrc
echo "## ROS-HACKS END ##" >>~/.bashrc

# Replace ~/.inputrc
if [[ -f ~/.inputrc ]];then
    mv ~/.inputrc ~/.inputrc.bak
fi
ln -nsf ${PWD}/inputrc ~/.inputrc

# Add crontab updater
crontab -l > mycron
echo "* 00 * * * $PWD/updater.sh" >> mycron
crontab mycron
rm mycron
