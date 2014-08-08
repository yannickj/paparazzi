#!/bin/bash

ADDR_BASE=192.168.1.

if [ `grep -c '\.' <<< $1` == 1 ]
then
  ADDR=$1
else
  ADDR=$ADDR_BASE$1
fi

pid=0

function quit {
  echo "Stop video"
  if [ $pid > 0 ]
  then
    kill -9 $pid
  fi
  exit 0
}

trap quit SIGINT

mkdir -p sdp_tmp/$ADDR
../ardrone2_drivers/ardrone2.py --host=$ADDR download_file sdp_tmp/$ADDR/x86_config-mjpeg.sdp sdp

if [ ! -f sdp_tmp/$ADDR/x86_config-mjpeg.sdp ];
then
  echo "Unable to download sdp file from $ADDR"
  exit 0
fi

echo "Start video"
while [ 1 ]
do
  /usr/bin/mplayer -really-quiet sdp_tmp/$ADDR/x86_config-mjpeg.sdp&
  pid=$!
  wait $pid
  echo "Restart video"
done

