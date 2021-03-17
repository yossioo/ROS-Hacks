#!/usr/bin/env bash

function prompt_new_ws() {
    printf "${LIGHT_BLUE_TXT}Creating new ROS workspace, specify ROS1 or ROS2 (enter 1 or 2):${NC}\n"
    num=0
    while [[ $num != [12] ]]; do
        read -n 1 -p "[1/2]: " num
        case $num in
            [1]*)
                type="ROS1"
                echo ""
            ;;
            [2]*)
                type="ROS2"
                echo ""
            ;;
            *)
                echo ""
                printf "${RED_TXT}Incorrect values selected. Enter 1 or 2:${NC}\n"
                # return 1
            ;;
        esac
        
    done
    printf "${LIGHT_BLUE_TXT}Please enter the WS name, will create ~/####_ws/src:${NC}\n"
    read -p ":  " name
    createWS $type $name
}

function createWS() {
    type=${1:-""}
    name=${2:-""}
    if [[ -z "${type}" ]]; then
        printf "${RED_TXT}ROS type not specified, specify ROS1 or ROS2.${NC}\n"
    else
        if [[ -z "${name}" ]]; then
            printf "${RED_TXT}ROS workspace name not specified.${NC}\n"
        else
            mkdir -p ~/${name}_ws/src
            set_current_ws ~/${name}_ws
            get_current_ws
            cd $curr_ws
            if [[ $type == "ROS1" ]]; then # Catkin found in ws
                csr
                catkin build
                source_ws ${curr_ws}
                elif [[ $type == "ROS2" ]]; then # Catkin found in ws
                csr2
                colcon build --symlink-install
                source_ws ${curr_ws}
            else
                printf "${RED_TXT}UNKNOWN ROS type.${NC}\n"
            fi
        fi
    fi
}

function unROS() {
    # Get all variables containing 'ROS' or 'ros'
    vars=$(env | egrep -i ROS | column -t -s '=' | sed -E 's/ .*//g')
    
    # For everyone do:
    for v in $vars; do
        # Get the value
        if [[ $v == *"PWD"* ]]; then
            continue
        fi
        str=$(printenv $v)
        # Divide into array separated by colon
        arrIN=(${str//:/ })
        # If variable name contains 'ROS' unset it
        if [[ $v == *"ROS"* ]]; then
            unset $v
            continue
        else # Otherwise evaluate the fields
            # For every field check:
            for i in "${arrIN[@]}"; do
                # If contains 'ros' or '_ws/' - skip the field
                if [[ $i == *"ros"* ]]; then
                    continue
                fi
                if [[ $i =~ "_ws/" ]]; then
                    continue
                fi
                # Save the current state of temporary variable
                old=$(printenv ${v}_tmp)
                if [ -z "$old" ]; then # If it is empty: add current field
                    export ${v}_tmp=${i}
                else # If it is not empty - append the field
                    export ${v}_tmp=$(printenv ${v}_tmp):${i}
                fi
            done
            # After all fields are processed - move the temp var to the original
            export ${v}=$(printenv ${v}_tmp)
            unset ${v}_tmp # Just to be sure, maybe not required
            if [[ -z "$(printenv ${v})" ]]; then
                unset $v
            fi
        fi
    done
    
    # Unset paths that are setup from sourcing ROS
    unset CMAKE_PREFIX_PATH
    unset AMENT_PREFIX_PATH
    unset COLCON_PREFIX_PATH
}

function select_ws() {
    get_current_ws
    find_ws
    print_domain_info
    ask_for_ws_and_domain
    
    c=0
    for i in "${arrIN[@]}"; do
        c=$(($c + 1))
        if [[ $c == $num ]]; then
            determine_ws_ros_version $i
            if [[ $ros_type == "ROS1" ]]; then # Catkin found in ws
                set_current_ws $i
                source_ws $i
                elif [[ $ros_type == "ROS2" ]]; then # Catkin found in ws
                set_current_ws $i
                source_ws $i
            else
                printf "${RED_TXT}MIXED ROS WS - Sourcing aborted.${NC}\n"
            fi
            
            return 0
        fi
    done
}

function get_current_ws() {
    unset curr_ws
    curr_ws=$(cat $WS_FILE)
    # printf "${GREEN_TXT2}ROS workspace: $curr_ws ${NC}\n"
}
function get_current_ws_name(){
    get_current_ws
    myarray=(${curr_ws//\// })
    
    echo ${myarray[-1]}
}
function set_current_ws() {
    new_curr_ws=${1:-""}
    if [[ -z "${new_curr_ws}" ]]; then
        printf "${RED_TXT}ROS workspace path not specified.${NC}\n"
    else
        echo $new_curr_ws >$WS_FILE
    fi
}

function get_ros_domain_id(){
    unset domain_id
    if [ -f "$ROS_DOMAIN_ID_FILE" ]; then
        domain_id=$(cat $ROS_DOMAIN_ID_FILE)
    fi
}

function print_ros_domain_id(){
    get_ros_domain_id
    echo $domain_id
}

function set_ros_domain_id(){
    id=${1:-""}
    if [[ -z "${id}" ]]; then
        printf "${RED_TXT}ROS DOMAIN ID not specified.${NC}\n"
    else
        printf "${BLUE}ROS DOMAIN ID${NC} set to: ${BLUE}$id${NC}\n"
        echo $id >$ROS_DOMAIN_ID_FILE
    fi
}

function source_ws() {
    ws_name=${1:-""}
    if [[ -z "${ws_name}" ]]; then
        printf "${RED_TXT}ROS workspace path not specified.${NC}\n"
    else
        determine_ws_ros_version $ws_name
        if [[ $ros_type == "ROS1" ]]; then # Catkin found in ws
            unROS
            printf "Sourcing ${WHITE_TXT} $ws_name ${NC}\n"
            source $ws_name/devel/setup.bash
            elif [[ $ros_type == "ROS2" ]]; then # Catkin found in ws
            unROS
            printf "Sourcing ${WHITE_TXT}$ws_name ${NC}\n"
            source $ws_name/install/setup.bash
        else
            printf "${RED_TXT}ERROR in ROS WS $ws_name - Sourcing aborted.${NC}\n"
        fi
    fi
    get_ros_domain_id
    if [[ ! -z "${domain_id}" ]]; then
        export ROS_DOMAIN_ID=$domain_id
    fi
}

function ask_for_ws_and_domain() {
    if [[ $(($ws_count)) -lt 10 ]]; then
        read -n 1 -p "Select desired WS # to do a clean source [1-$ws_count] or 'd' to change domain : " num
    else
        read -n 2 -p "Select desired WS ## to do a clean source [1-$ws_count] or 'd' to change domain : " num
    fi
    case $num in
        [123456789]*)
            echo ""
            # echo "Sourcing WS #$num"
        ;;
        [Dd]*)
            echo ""
            printf "Please enter new ${BLUE_TXT}ROS_DOMAIN_ID${NC} (empty to disable): "
            read new_domain
            if [[ -z "${new_domain}" ]]; then
                echo "Disabling domain"
                rm $ROS_DOMAIN_ID_FILE
                unset ROS_DOMAIN_ID
            else
                set_ros_domain_id $new_domain
                export ROS_DOMAIN_ID=$new_domain
            fi
            return 0
        ;;
        *)
            echo ""
            echo "Cancelling."
        ;;
    esac
}

function rebuild_curr_ws() {
    get_current_ws
    determine_ws_ros_version $curr_ws
    if [[ $ros_type == "ROS1" ]]
    then # Catkin found in ws
        (cd $curr_ws && source devel/setup.bash  && cab $1)
        if [[ $? == 0 ]]; then
            source_ws $curr_ws
        else
            printf "${RED_TXT}catkin errors - sourcing aborted.${NC}\n"
        fi
    elif [[ $ros_type == "ROS2" ]]
    then # Catkin found in ws
        (unROS && cd $curr_ws && source install/setup.bash && cob $1)
        if [[ $? == 0 ]]; then
            source_ws $curr_ws
        else
            printf "${RED_TXT}catkin errors - sourcing aborted.${NC}\n"
        fi
    else
        printf "${RED_TXT}ERROR in ROS WS $ws_name - Sourcing aborted.${NC}\n"
    fi
    
}

function find_ws() {
    ws=$(find ~/ -maxdepth 1 -type d -name \*ws\* | sort)
    # echo $ws
    arrIN=(${ws// / })
    # echo ${arrIN[2]}
    printf "${GREEN_TXT}Workspaces in ~${NC}\n"
    # Search for longest ws name
    max_l=0
    for i in "${arrIN[@]}"; do
        l=$(expr length "$i")
        # echo $(($l)), $(($max_l))
        if [[ $(($l)) -gt $(($max_l)) ]]; then
            # echo "New line is longer"
            max_l=$(($l))
        fi
    done
    ws_count=0
    pad=""
    c=$(($max_l + 2))
    while [[ $c > 0 ]]; do
        pad="$pad─"
        c=$((c - 1))
    done
    
    printf "┌%-5s┬%-10s┬%-$(echo $max_l)s┐\n" "─────" "──────────" "$pad"
    printf "│ %-3s │ %-8s │ %-$(echo $max_l)s │\n" "NUM" "ROS_TYPE" "LOCATION"
    printf "├%-5s┼%-10s┼%-$(echo $max_l)s┤\n" "─────" "──────────" "$pad"
    for i in "${arrIN[@]}"; do
        if [[ $i == $curr_ws ]]; then
            sourced_color=${WHITE_TXT}
        else
            sourced_color=${NC}
        fi
        
        determine_ws_ros_version $i
        ws_count=$(($ws_count + 1))
        l=$(expr length "$i")
        
        if [[ $ros_type == "ROS1" ]]; then # Catkin found in ws
            printf "│ ${NC}%-3s${NC} │ ${BLUE_TXT}%-8s${NC} │ ${sourced_color}%-$(echo $max_l)s${NC} │\n" "$ws_count" "$ros_type" "$i"
            elif [[ $ros_type == "ROS2" ]]; then # Catkin found in ws
            printf "│ ${NC}%-3s${NC} │ ${GREEN_TXT2}%-8s${NC} │ ${sourced_color}%-$(echo $max_l)s${NC} │\n" "$ws_count" "$ros_type" "$i"
        else
            printf "│ ${NC}%-3s${NC} │ ${RED_TXT}%-8s${NC} │ ${sourced_color}%-$(echo $max_l)s${NC} │\n" "$ws_count" "$ros_type" "$i"
        fi
        
    done
    printf "└%-5s┴%-10s┴%-$(echo $max_l)s┘\n" "─────" "──────────" "$pad"
    
}

function print_domain_info(){
    get_ros_domain_id
    printf "${BLUE_TXT}ROS_DOMAIN_ID${NC}: ${LIGHT_BLUE_TXT}$domain_id${NC}\n"
}

function determine_ws_ros_version() {
    ws=${1:-""}
    catkinws=0
    colconws=0
    if [[ -z "${ws}" ]]; then
        printf "${RED_TXT}ROS workspace path not specified.${NC}\n"
    else
        # printf "${BLUE_TXT}Checking WS: ${ws}.${NC}\n"
        if [[ -f "${ws}/.catkin_tools/CATKIN_IGNORE" ]]; then
            # echo CATKIN_IGNORE FOUND
            catkinws=1
        fi
        if [[ -f "${ws}/build/CATKIN_IGNORE" ]]; then
            # echo CATKIN_IGNORE FOUND
            catkinws=1
        fi
        if [[ -f "${ws}/build/COLCON_IGNORE" ]]; then
            # echo CATKIN_IGNORE FOUND
            colconws=1
        fi
        # echo CATKIN, COLCON = $catkinws, $colconws
        if [[ $catkinws -eq "1" ]]; then # Catkin found in ws
            ros_type='ROS1'
            if [[ $colconws -eq "1" ]]; then # COLCON found in ws
                ros_type='MIXED'
            fi
        else                               # Catkin not found in ws
            if [[ $colconws -eq "1" ]]; then # COLCON found in ws
                ros_type='ROS2'
            else
                ros_type='ERR'
                
            fi
        fi
    fi
}

function ask_for_num() {
    max=${1:-"1"}
    if [[ $(($max)) -lt 10 ]]; then
        read -n 1 -p "Select a number of file to edit [1-$max] or cancel : " num
    else
        read -n 2 -p "Select a number of file to edit [1-$max] or cancel : " num
    fi
    case $num in
        [123456789]*)
            echo ""
            # echo "Sourcing WS #$num"
        ;;
        # [Cc]*)
        #   echo ""
        #   return 1
        #   ;;
        *)
            echo ""
            echo "Cancelling."
        ;;
    esac
}

# Function sets ROS_MASTER_URI variable. Either specify last byte for Mobilicom pool, or specify full ip and second argumen 1
function RM() {
    ip=${1:-100}
    full_ip=${2:-0}
    if [ ${full_ip} != 1 ]; then
        ip="192.168.131.${ip}"
    fi
    arg="ROS_MASTER_URI=http://${ip}:11311"
    printf "Setting environment variable: ${arg}\n"
    export ${arg}
}

function rt() {
    topic_filter=${1:-0}
    if [ ${topic_filter} == 0 ]; then
        ros2 topic list
    else
        ros2 topic list | grep ${topic_filter}
    fi
}

function fixJB() {
    CLION_FILE=~/.local/share/applications/jetbrains-clion.desktop
    PYCHARM_FILE=~/.local/share/applications/jetbrains-pycharm.desktop
    PYCHARM_CE_FILE=~/.local/share/applications/jetbrains-pycharm-ce.desktop
    if [ -f "$CLION_FILE" ]; then
        printf "${GREEN_TXT}Patching Clion shortcut.${NC}\n"
        sed -i -e 's/Exec="/Exec=bash -i -c "/g' $CLION_FILE
        sed -i -e 's/Name=CLion/Name=ROS flavored CLion/g' $CLION_FILE
    else
        printf "${YELLOW_TXT}Clion not found in ~/.local/share/applications${NC}\n"
    fi
    
    if [ -f "$PYCHARM_FILE" ]; then
        printf "${GREEN_TXT}Patching Pycharm Professional shortcut.${NC}\n"
        sed -i -e 's/Exec="/Exec=bash -i -c "/g' $PYCHARM_FILE
        sed -i -e 's/Name=PyCharm Professional/Name=ROS flavored PyCharm Professional/g' $PYCHARM_FILE
    else
        printf "${YELLOW_TXT}PyCharm Professional not found in ~/.local/share/applications${NC}\n"
    fi
    
    if [ -f "$PYCHARM_CE_FILE" ]; then
        printf "${GREEN_TXT}Patching PyCharm CE shortcut.${NC}\n"
        sed -i -e 's/Exec="/Exec=bash -i -c "/g' $PYCHARM_CE_FILE
        sed -i -e 's/Name=PyCharm /Name=ROS flavored PyCharm /g' $PYCHARM_CE_FILE
    else
        printf "${YELLOW_TXT}PyCharm CE not found in ~/.local/share/applications${NC}\n"
    fi
}

### Arducopter TMUX:

function clean_ROS2_ws() {
    ws=${1:-""}
    if [[ -z "${ws}" ]]; then
        printf "${RED_TXT}ROS workspace path not specified.${NC}\n"
    else
        printf "${YELLOW_TXT}Clearing the workspace: ${ws} ${NC}  "
        find ${ws}/log -mindepth 1 -maxdepth 1 -type d -print0 | xargs -0 rm -R >/dev/null 2>&1
        find ${ws}/install -mindepth 1 -maxdepth 1 -type d -print0 | xargs -0 rm -R >/dev/null 2>&1
        find ${ws}/build -mindepth 1 -maxdepth 1 -type d -print0 | xargs -0 rm -R >/dev/null 2>&1
        printf "${GREEN_TXT}---DONE---${NC}\n"
    fi
    
}

function roscd() {
    pkgname=${1:-""}
    if [[ -z "${pkgname}" ]]; then
        printf "${RED_TXT}ROS package name not specified.${NC}\n"
    else
        get_current_ws
        determine_ws_ros_version $curr_ws
        if [[ $ros_type == "ROS2" ]]; then # Catkin found in ws
            cd $(ros2 pkg prefix $pkgname)
        else
            roscd $pkgname
        fi
        
    fi
    
}

