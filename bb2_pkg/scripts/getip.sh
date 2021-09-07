# This script is just for reference. 
# Please use iptags.sh to launch Bebop's master node and april tag controller.
# Please use ipfly.sh to launch Bebop's master node and operator controls.
hostname=${1:-Bebop2_A}
echo $(host $hostname | egrep -o '[0-9.]{7,15}')
