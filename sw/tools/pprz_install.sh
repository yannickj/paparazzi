#!/bin/bash

#Initialize the major and minor variables
Major=4
Minor=0
FindMaj=0
FindMin=0

#Add the personal package archive of the paparazzi-uav project
sudo add-apt-repository -y ppa:paparazzi-uav/ppa
if [ $? = "0" ] 
then	
	echo "Paparazzi-uav ppa added to the source list."
else
	echo "The command $sudo add-apt-repository have failed (try to solve the problem on this command first)."
fi

#Add the personal package archive of the team-gcc-arm-embedded (required for paparazzi)
sudo add-apt-repository -y ppa:team-gcc-arm-embedded/ppa
if [ $? = "0" ] 
then	
	echo "team-gcc-arm-embedded ppa added to the source list."
else
	echo "The command $sudo add-apt-repository have failed (try to solve the problem on this command first)."
fi

#Get the update from the ppa
sudo apt-get update
if [ $? = "0" ] 
then	
	echo "PPA up to date."
else
	echo "PPA update fail."
fi

sudo apt-get -f -y install paparazzi-dev paparazzi-jsbsim gcc-arm-embedded
if [ $? = "0" ] 
then	
	echo "Installation of the dependencies for the paparazzi software succeed."
else
	echo "Installation of the dependencies fail."
fi

#Return to home
cd ~

#Clone the git master repository of the paparazzi project
git clone --origin upstream https://github.com/paparazzi/paparazzi.git
if [ $? = "0" ] 
then	
	echo "Master repository of the paparazzi project cloned successfully."
else
	echo "Cloning the master repository of the paparazzi project have failed."
fi

#Go to cloned repository
cd ~/paparazzi

########################################################################
#git branch -r ou git tag -r ?
if [ -z $1 ]
then
	echo "No parameters detected. The latest stable version will be installed."
	while [ $FindMaj -eq 0]
		do 
			git branch | grep v$Major.
			if [ $? = "0" ] 
				then	
					echo "Major number validate."
					Major=$(($Major+1))
				else
					echo "Major number does not exist."
					echo "Latest Major version is the $Major."
					FindMaj=1
			fi
	done
	while [ $FindMin -eq 0]
		do 
			git branch | grep v$Major.$Minor
			if [ $? = "0" ] 
				then	
					echo "Minor number validate."
					Major=$(($Major+2))
				else
					echo "Minor number does not exist."
					echo "Latest Minor version is the $Minor."
					FindMin=1
			fi
	done
	echo "The lastest version is the v$Major.$Minor."
	git checkout -b v$Major.$Minor upstream/v$Major.$Minor
else
	echo "Parameters detected."
	if [ $1 = "dev" ] || [ $1 = "master" ]
	then
		echo "You'r now in the master branch!"
	elif [ $1 = "branch" ]
	then
		git checkout -b $2 upstream/$2
		if [ $? = "0" ] 
		then	
			echo "You'r now in the $2 branch!"
		else
			echo "The git checkout command have failed."
		fi
	fi
fi
#####################################################################

sudo cp conf/system/udev/rules/*.rules /etc/udev/rules.d/
if [ $? = "0" ] 
then	
	echo "Rules for the system copyed in your /etc files."
else
	echo "The rules for the system didn't succeed to be copyed in your laptop."
fi

sudo udevadm control --reload-rules
if [ $? = "0" ] 
then	
	echo "Reaload the rules suceed."
else
	echo "Reload the rules failed"
fi

make
if [ $? = "0" ] 
then	
	echo "Make OK!"
else
	echo "Make fail!"
fi

echo "Installation finished, now the paparazzi software will be launched!"

#Execute paparazzi
./paparazzi