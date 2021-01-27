#!/usr/bin/env bash
if [[ ${SKIP} == 1 ]]; then
    return 1
fi

ROS2_NAME='foxy'
if [[ $(lsb_release -cs) == 'xenial' ]]; then
    ROS1_NAME='kinetic'
elif [[ $(lsb_release -cs) == 'bionic' ]]; then
    ROS1_NAME='melodic'
    ROS2_NAME='dashing'
fi


# Define colors:
NC='\033[0m'
GREEN_TXT='\e[0;32m'
GREEN_TXT2='\e[32m'
DARK_GREEN_TXT='\e[2;32m'
WHITE_TXT='\e[1;37m'
RED_TXT='\e[31m'
DIM_RED_TXT='\e[2;31m'
LIGHT_BLUE_TXT='\e[96m'
BLUE_TXT='\e[34m'
DIM_BLUE_TXT='\e[2;34m'
DARK_GREY_TXT='\e[90m'
YELLOW_TXT='\e[93m'
BOLDRED="\033[1m\033[31m"     # /* Bold Red */
BOLDGREEN="\033[1m\033[32m"   # /* Bold Green */
BOLDYELLOW="\033[1m\033[33m"  # /* Bold Yellow */
BOLDBLUE="\033[1m\033[34m"    # /* Bold Blue */
BOLDMAGENTA="\033[1m\033[35m" # /* Bold Magenta */
BOLDCYAN="\033[1m\033[36m"    # /* Bold Cyan */
BOLDWHITE="\033[1m\033[37m"   # /* Bold White */

WS_FILE=$HOME/.ros_ws_selected
ROS_DOMAIN_ID_FILE=$HOME/.ros_domain_id
QUICK_COMMAND_FILE=.quick_command

alias pR='printenv | grep -i -e ROS -e CATKIN -e CMAKE -e RMW'
alias sw='source_ws $(cat $WS_FILE)'
alias sr='source /opt/ros/${ROS1_NAME}/setup.bash'
alias csr='unROS; source /opt/ros/${ROS1_NAME}/setup.bash'
alias sr2='source /opt/ros/${ROS2_NAME}/setup.bash'
alias csr2='unROS; source /opt/ros/${ROS2_NAME}/setup.bash'

alias cab='catkin build --summary'
alias cob='colcon build --symlink-install'
alias cobd='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug'
alias cobr='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'
alias cobp='colcon build --symlink-install --packages-select'
alias cobput='colcon build --symlink-install --packages-up-to'
alias cobpv='colcon build --symlink-install --event-handlers console_cohesion+ --packages-select'
alias coc='clean_ROS2_ws $(cat $WS_FILE)'
alias cw='cd $(cat $WS_FILE)'
alias cs='cd $(cat $WS_FILE)/src'
alias cg='touch COLCON_IGNORE'
alias rcg='rm COLCON_IGNORE'

alias rt='ros2 topic'
alias rn='ros2 node'
alias rb='ros2 bag'
alias whgit='git config --get remote.origin.url'
alias o='sudo chown -R $USER:$USER '
alias x='chmod +x'
alias mke='make -j`nproc`'
alias temp='watch -n 0.1 sensors'

# shopt -s direxpand
shopt -s expand_aliases
shopt -s histappend
shopt -s cmdhist
export PROMPT_COMMAND='history -a'
# export HISTFILESIZE=1000000
# export HISTSIZE=1000000
export HISTCONTROL=ignoreboth
export HISTIGNORE='t:f:l:ls:bg:fg:history:h:select_ws:kill-tmux-gz:test-launch:qe-file:kp:f_run_icl'
export HISTTIMEFORMAT='%F %T '

# PS1=' \[\e[1;32m\]\u\[\033[00m\] \[\e[32m\]$(get_current_ws_name):$ROS_DOMAIN_ID\[\033[00m\] \[\033[03;94m\]\w\[\033[00m\]\[\033[38;5;51m\]$(__git_ps1)\[\033[00m\]:\n\$ '

bind '"^[OS":"~/.ROS-Hacks/ROS2-Utilities/diag-node-printer.py\n"'
bind '"^[[1;2S":"~/.ROS-Hacks/ROS2-Utilities/diag-node-printer.py target:=mc\n"'
bind '"\eOS":"~/.ROS-Hacks/ROS2-Utilities/diag-node-printer.py\n"'
bind '"\e[1;2S":"~/.ROS-Hacks/ROS2-Utilities/diag-node-printer.py target:=mc\n"'