#!/bin/sh

processes="vision
motion"
while [ 1 ]
do
  for p in $processes 
  do
    if [ ! "$(pidof $p)" ] 
    then
      echo -e "\n\n***********\nRESTARTING $p PROCESS\n**********\n"
      /home/nao/bin/$p &
    fi
  done
  sleep 1
done
