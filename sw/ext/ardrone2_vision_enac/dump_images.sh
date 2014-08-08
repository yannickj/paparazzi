#!/bin/bash

ADDR_BASE=192.168.1.

if [ `grep -c '\.' <<< $1` == 1 ]
then
  ADDR=$1
else
  ADDR=$ADDR_BASE$1
fi

mkdir -p images_tmp/$ADDR
../ardrone2_drivers/ardrone2.py --host=$ADDR download_dir images_tmp/$ADDR images

