#!/bin/bash

testMajor=0
testMinor=0
Major=4
Minor=0
var="v$Major.$Minor"
fin=50
version="v5.10"

if [ -z $1 ]
then
	echo "Pas de paramètre"
else
	echo "Paramètre ok"
	echo ${var::2}
	if [ $1 = "dev" ] || [ $1 = "master" ]
	then
		while [ $testMajor -eq 0 ]
		do
			if [ "${var::1}" = "${version::2}" ]
			then
				test=1
			else
				echo $var
				var=$(($var+1))
				echo $var
			fi
		done
		while [ $testMinor -eq 0 ]
		do

		done
	else
		echo "not ok"
	fi
fi

