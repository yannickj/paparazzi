#!/bin/bash

ADDR_BASE=192.168.1.

if [ `grep -c '\.' <<< $1` == 1 ]
then
  ADDR=$1
  PORT=$2
else
  ADDR=$ADDR_BASE$1
  PORT=5$1
fi

pid=0

echo "Start video for $ADDR on port $PORT"
/usr/bin/avplay -loglevel quiet -max_delay 50 -fflags nobuffer rtp://$ADDR:$PORT

