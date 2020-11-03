#!/bin/bash

bold=$(tput bold)
normal=$(tput sgr0)
opt=-1

#External connection flags
kuka=false
vrep=false
forcedimension=false
#NeueBot flags
corekuka=false
corevrep=false
core=false
app=false
#Haptic flags
haptickuka=false
hapticvrep=false
haptic=false

clear_deployers() {
	read -p "Clear previous deployers? y/N " ans
	case $ans in
		y|Y) sudo pkill MainThread; sudo pkill clangbackend;;
		n|N) ;;
		*) ;;
	esac
	echo " "
}

check_kuka_ip() {
	printf "Host configured to connect to KUKA's IP address... " 
	currentIPaddress=$(ip addr show eth0 | grep "inet\b" | awk '{print $2}' | cut -d/ -f1)
	if [ "$currentIPaddress" = "192.170.10.100" ]
	then
		printf $bold"OK\n"$normal
		kuka=true
	else
		printf "\e[31mFail\n\e[39m"
	fi
}

check_arch_ip() {
	printf "Host configured to connect to KUKA's IP address... " 
	currentIPaddress=$(ip addr show eth0 | grep "inet\b" | awk '{print $2}' | cut -d/ -f1)
	if [ "$currentIPaddress" = "192.170.10.15" ]
	then
		printf $bold"OK\n"$normal
		arch=true
	else
		printf "\e[31mFail\n\e[39m"
	fi
}

check_vrep() {
	printf "V-REP running... " 
	if pgrep -x "vrep" > /dev/null 
	then
		printf $bold"OK\n"$normal
		vrep=true
	else
		printf "\e[31mFail\n\e[39m"
	fi
}

check_force_dimension() {
	printf "Force Dimension Haptic Omega 7 connected... " 
	forceDimensionID=$(lsusb | grep "ID 1451:0402" | awk '{print $6}')
	if [ "$forceDimensionID" = "1451:0402" ]
	then
		printf $bold"OK\n"$normal
		forcedimension=true
	else
		printf "\e[31mFail\n\e[39m"
	fi
}

check_core_deployer() {
	printf "Core Layer deployer running... " 
	if pgrep -x "MainThread" > /dev/null && "$corekuka" = true || "$corevrep" = true  
	then
		printf $bold"OK\n"$normal
		core=true
	else
		printf "\e[31mFail\n\e[39m"
	fi
}

check_app_deployer() {
	printf "Application Layer deployer is running... " 
	if pgrep -x "MainThread" > /dev/null && [ "$app" = true ]
	then
		printf $bold"OK\n"$normal
		core=true
	else
		printf "\e[31mFail\n\e[39m"
	fi
}

check_haptic_deployer() {
	printf "Haptic Control Layer deployer is running... " 
	if pgrep -x "MainThread" > /dev/null && "$haptickuka" = true || "$hapticvrep" = true
	then
		printf $bold"OK\n"$normal
		haptic=true
	else
		printf "\e[31mFail\n\e[39m"
	fi
}

clear
# Black background & Yellow text \e[40m\e[92m
echo -e $bold"\e[43m\e[30m    _   ___   _ _   __  ___    _                            _                   "
echo "   | | / / | | | | / / / _ \  | |                          | |                  "
echo "   | |/ /| | | | |/ / / /_\ \ | |     __ _ _   _ _ __   ___| |__   ___ _ __     "
echo "   |    \| | | |    \ |  _  | | |    / _  | | | | '_ \ / __| '_ \ / _ \ '__|    "
echo "   | |\  \ |_| | |\  \| | | | | |___| (_| | |_| | | | | (__| | | |  __/ |       "
echo "   \_| \_/\___/\_| \_/\_| |_/ \_____/\__,_|\__,_|_| |_|\___|_| |_|\___|_|       "                                               
echo "                                                                                "
echo "   The applications for each of the variants 'NeueBot' and 'Haptic' must be     "
echo "   executed sequentially. Make sure the preceeding options are running prior    "
echo "   to any selection. If you are running in a simulation environment make        "
echo "   sure you launch the V-REP simulator and start the simulation                 "
echo "================================================================================"
echo -e "\e[49m\e[92m" # Default background

clear_deployers

while [ $opt != "0" ] 
do
	sleep 1
	echo -e $bold"\e[96mSelect an application to run:" $normal
	echo -e "\e[96m(Add a 'q' before the option to quickstart the application)" $normal
	echo -e $bold"[1]" $normal"NeueBot Interface Layer to connect to KUKA controller."
	echo -e $bold"[2]" $normal"NeueBot Core Layer to control the Interface Layer - FRI."
	echo -e $bold"[3]" $normal"NeueBot Core Layer to control the V-REP simulation."
	echo -e $bold"[4]" $normal"NeueBot Application Layer Architecture."
	echo -e $bold"[5]" $normal"NeueBot Graphical User Interface."
	echo -e $bold"[6]" $normal"Haptic Core Layer to control the KUKA robot."
	echo -e $bold"[7]" $normal"Haptic Core Layer to control the V-REP simulation."
	echo -e $bold"[8]" $normal"Haptic Graphical User Interface."
	echo -e $bold"[0]" $normal"Exit."
	read -p "Option: "$bold opt
	echo $normal" "
	case $opt in
		1) check_kuka_ip
				if [ "$kuka" = true ]
				then				
					echo "Running the NeueBot Interface Layer..." 
					(cd ${HOME}/neuebot_ws/src/Launchers
					gnome-terminal --geometry 128x20+0+0 -e "bash -i -c 'deployer-corba -s "KUKALauncher.ops" -l Warning; bash'" %F &>/dev/null &)  
					corekuka=true
				else
					echo "Cannot run the NeueBot Core Layer to control the real robot..." 
				fi
			;;
		q1)	(cd ${HOME}/neuebot_ws/src/Launchers
					gnome-terminal --geometry 128x20+0+0 -e "bash -i -c 'deployer-corba -s "KUKALauncher.ops" -l Warning; bash'" %F &>/dev/null &)   
					corekuka=true
			;;	
		2) check_arch_ip
				if [ "$arch" = true ]
				then				
					echo "Running the NeueBot Core Layer..." 
					(cd ${HOME}/neuebot_ws/src/Launchers
					gnome-terminal --geometry 128x20+0+0 -e "bash -i -c 'deployer-corba -s "ArchLauncher.ops" -l Warning; bash'" %F &>/dev/null &)  
					corekuka=true
				else
					echo "Cannot run the NeueBot Core Layer..." 
				fi
			;;
		q2)	(cd ${HOME}/neuebot_ws/src/Launchers
					gnome-terminal --geometry 128x20+0+0 -e "bash -i -c 'deployer-corba -s "ArchLauncher.ops" -l Warning; bash'" %F &>/dev/null &)   
					corekuka=true
			;;	
		r2) check_arch_ip
				if [ "$arch" = true ]
				then				
					echo "Running the NeueBot Core Layer..." 
					(cd ${HOME}/neuebot_ws/src/Launchers
					gnome-terminal --geometry 128x20+0+0 -e "bash -i -c 'deployer-corba -s "ArchLauncher.ops" -l Warning; bash'" %F &>/dev/null &)  
					corekuka=true
				else
					echo "Cannot run the NeueBot Core Layer..." 
				fi
			;;
		3) check_vrep 
				if [ "$vrep" = true ] 
				then
					echo "Running the NeueBot Core Layer to control a virtual robot. Make sure the simulation is started..."  
					(cd ${HOME}/neuebot_ws/src/Launchers
					 gnome-terminal --geometry 128x20+0+0 -e "bash -i -c 'deployer-corba -s "ArchSimLauncher.ops" -l Warning; bash'" %F &>/dev/null &)
					corevrep=true
				else 
					echo "Cannot run the NeueBot Core Layer to control a virtual robot."				
				fi				
			;;	
		q3) (cd ${HOME}/neuebot_ws/src/Launchers
				 	gnome-terminal --geometry 128x20+0+0 -e "bash -i -c 'deployer-corba -s "ArchSimLauncher.ops" -l Warning; bash'" %F &>/dev/null &)  
					corevrep=true
			;;
		4) check_core_deployer
				if [ "$core" = true ]
				then
					echo "Running the NeueBot Application Layer..." 
					(cd ${HOME}/neuebot_ws/src/Launchers
					 gnome-terminal --geometry 128x20+0+395 -e "bash -i -c 'deployer-corba -s "ApplicationLauncher.ops" -l Warning; bash'" %F &>/dev/null &)  
					app=true
				else 
					echo "Cannot run the NeueBot Application Layer."				
				fi
			;;		
		q4)  (cd ${HOME}/neuebot_ws/src/Launchers
					 gnome-terminal --geometry 128x20+0+395 -e "bash -i -c 'deployer-corba -s "ApplicationLauncher.ops" -l Warning; bash'" %F &>/dev/null &)   
					 app=true
			;;	
		r4) check_core_deployer
				if [ "$core" = true ]
				then
					echo "Running the NeueBot Application Layer..." 
					(cd ${HOME}/neuebot_ws/src/Launchers
					 gnome-terminal --geometry 128x20+0+395 -e "bash -i -c 'deployer-corba -s "ApplicationLauncher.ops" -l Warning; bash'" %F &>/dev/null &)  
					app=true
				else 
					echo "Cannot run the NeueBot Application Layer."				
				fi
			;;		
		5) check_core_deployer
				check_app_deployer
				if [ "$app" = true ]
				then
					echo "Running the NeueBot Graphical User Interface..." 
					(cd ${HOME}/neuebot_ws/build/test_gui/debug; ./test_gui &) 2>/dev/null 
					cd ${HOME}/neuebot_ws/src/Launchers
				else 
					echo "Cannot run the NeueBot Graphical User Interface."				
				fi
			;;		
		r5) check_core_deployer
				check_app_deployer
				if [ "$app" = true ]
				then
					echo "Running the NeueBot Graphical User Interface..." 
					(cd ${HOME}/neuebot_ws/build/test_gui/release; ./test_gui &) 2>/dev/null 
					cd ${HOME}/neuebot_ws/src/Launchers
				else 
					echo "Cannot run the NeueBot Graphical User Interface."				
				fi
			;;		
		q5) (cd ${HOME}/neuebot_ws/build/test_gui/debug; ./test_gui &) 2>/dev/null 
					cd ${HOME}/neuebot_ws/src/Launchers
			;;
		6) check_kuka_ip
				check_force_dimension
				if "$kuka" = true && "$forcedimension" = true
				then				
					echo "Running the Haptic Core Layer..." 
					(cd ${HOME}/HapticWorkspace/src/Launchers
					gnome-terminal -e "bash -i -c 'deployer-corba -s "KUKALauncherRobot.ops" -l Warning; bash'" %F &>/dev/null &)  
					haptickuka=true
				else
					echo "Cannot run the Haptic Core Layer..." 
				fi
			;;	
		q6) (cd ${HOME}/HapticWorkspace/src/Launchers
					gnome-terminal -e "bash -i -c 'deployer-corba -s "KUKALauncherRobot.ops" -l Warning; bash'" %F &>/dev/null &)  
					haptickuka=true
			;;
		7) check_vrep 
				check_force_dimension
				if "$vrep" = true && "$forcedimension" = true
				then
					echo "Running the Haptic Core Layer to control a virtual robot. Make sure the simulation is started..."  
					(cd ${HOME}/HapticWorkspace/src/Launchers
					 gnome-terminal -e "bash -i -c 'deployer-corba -s "KUKALauncherTest.ops" -l Warning; bash'" %F &>/dev/null &)  
					hapticvrep=true
				else 
					echo "Cannot run the Haptic Core Layer to control a virtual robot."				
				fi				
			;;
		q7) (cd ${HOME}/HapticWorkspace/src/Launchers
					 gnome-terminal -e "bash -i -c 'deployer-corba -s "KUKALauncherTest.ops" -l Warning; bash'" %F &>/dev/null &)  
					hapticvrep=true
			;;
		8) check_haptic_deployer
				if [ "$haptic" = true ] 
				then
					echo "Running the Haptic Graphical User Interface..." 
						(cd ${HOME}/HapticWorkspace/build/haptic_gui/debug; ./haptic_gui &) 2>/dev/null 
						cd ${HOME}/HapticWorkspace/src/Launchers
				else
					echo "Cannot run the Haptic Graphical User Interface."			
				fi
			;;
		r8) check_haptic_deployer
				if [ "$haptic" = true ] 
				then
					echo "Running the Haptic Graphical User Interface..." 
						(cd ${HOME}/HapticWorkspace/build/haptic_gui/release; ./haptic_gui &) 2>/dev/null 
						cd ${HOME}/HapticWorkspace/src/Launchers
				else
					echo "Cannot run the Haptic Graphical User Interface."			
				fi
			;;
		q8) (cd ${HOME}/HapticWorkspace/build/haptic_gui/debug; ./haptic_gui &) 2>/dev/null 
						cd ${HOME}/HapticWorkspace/src/Launchers
			;;		
		0) echo "Done..." ;;
		*) echo "Enter a valid option."
	esac
	echo " "
done