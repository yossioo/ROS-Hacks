#!/bin/bash

source aliases.sh
printf "${LIGHT_BLUE_TXT}Installing ROS-Hacks${NC}.\n"

echo "source ${PWD}/aliases.sh" >>~/.bashrc
echo "source ${PWD}/functions.sh" >>~/.bashrc
echo "source_ws $(cat $WS_FILE)" >>~/.bashrc
mv ~/.inputrc ~/.inputrc.bak
ln -nsf ${PWD}/inputrc ~/.inputrc
