#!/bin/bash

cd `dirname $0`
set -eu

#read -p 'Enter team number for blue (default is 0): ' readBlue
#read -p 'Enter team number for red (default is 0): ' readRed

#declare -i blue=readBlue
#declare -i red=readRed
declare broadcast="192.168.1.255"

echo "Starting SPL GameController, blue will kick off"
#echo "Starting SPL GameController, team ${blue} plays in blue and team ${red} plays in red"
if [ -n "${1:-""}" ]; then
  broadcast="-broadcast ${1}"
  echo "Broadcasting to subnet ${1}"
fi

java -jar GameController.jar -b ${broadcast}
