#!/usr/bin/env bash
if [[ ${SKIP} == 1 ]]; then
    return 1
fi

if [[ $(lsb_release -cs) == 'xenial' ]]; then
    ROS1_NAME='kinetic'
    elif [[ $(lsb_release -cs) == 'bionic' ]]; then
    ROS1_NAME='melodic'
fi

ROS2_NAME='dashing'

# Define colors:
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
NC='\033[0m'

WS_FILE=$HOME/.ros_ws_selected
ROS_DOMAIN_ID_FILE=$HOME/.ros_domain_id
QUICK_COMMAND_FILE=.quick_command

alias pR='printenv | grep -i -e ROS -e CATKIN -e CMAKE -e RMW'
alias sc='source_ws $(cat $WS_FILE)'
alias sr='source /opt/ros/${ROS1_NAME}/setup.bash'
alias csr='unROS; source /opt/ros/${ROS1_NAME}/setup.bash'
alias sr2='source /opt/ros/${ROS2_NAME}/setup.bash'
alias csr2='unROS; source /opt/ros/${ROS2_NAME}/setup.bash'

alias cab='catkin build --summary'
alias cob='colcon build --symlink-install'
alias cobp='colcon build --symlink-install --event-handlers console_cohesion+ --packages-select '
alias coc='clean_ROS2_ws $(cat $WS_FILE)'
alias cw='cd $(cat $WS_FILE)'
alias cs='cd $(cat $WS_FILE)/src'

alias rt='ros2 topic '
alias rn='ros2 node '
alias rb='ros2 bag '
alias whgit='git config --get remote.origin.url'
alias o='sudo chown -R $USER:$USER '
alias x='chmod +x '
alias mke='make -j`nproc`'
alias temp='watch -n 0.1 sensors'

# shopt -s direxpand
shopt -s expand_aliases
shopt -s histappend
shopt -s cmdhist
export PROMPT_COMMAND='history -a'
export HISTFILESIZE=1000000
export HISTSIZE=1000000
export HISTCONTROL=ignoreboth
export HISTIGNORE='t:f:l:ls:bg:fg:history:h:select_ws:kill-tmux-gz:test-launch:qe-file:kp:kill-tmux-quick-command:exec-quick-command'
export HISTTIMEFORMAT='%F %T '


# PS1=' \[\e[1;32m\]\u\[\033[00m\] \[\e[32m\]$(get_current_ws_name):$ROS_DOMAIN_ID\[\033[00m\] \[\033[03;94m\]\w\[\033[00m\]\[\033[38;5;51m\]$(__git_ps1)\[\033[00m\]:\n\$ '
