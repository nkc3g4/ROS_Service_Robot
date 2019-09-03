#!/bin/bash

#################################################
# Author: www.corvin.cn
#################################################
# Description: 执行该脚本就可以配置好运行环境,
#   省去自己单独配置的麻烦过程。当编译完成源码后 
#   就需要执行该脚本,该脚本只需要执行一次即可。
#################################################
# History:
#    20180126:init this file.
#
#################################################

CURRENT_PATH=$(dirname $(readlink -f "$0"))
OMMIT_PATH="src/carebot_bringup/scripts"
WORKSPACE_PATH=${CURRENT_PATH%${OMMIT_PATH}}
green="\e[32;1m"
normal="\e[0m"

echo -e "${green}Get project workSpace path: ${WORKSPACE_PATH} ${normal}"

echo -e "\n${green} 0x00: set source project's devel/setup.bash to .bashrc file${normal}"
cd ${WORKSPACE_PATH}
source devel/setup.bash
echo "#config omniWheelCareRobot project env">>~/.bashrc
echo "source ${WORKSPACE_PATH}devel/setup.bash">>~/.bashrc


exit 0

