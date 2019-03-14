#!/bin/bash

export EDITOR="emacs" 
export PAPARAZZI_HOME=`pwd`
export https_proxy="proxy:3128" 
export http_proxy="proxy:3128" 
./paparazzi >& /dev/null &
