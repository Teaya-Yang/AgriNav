#!/bin/bash
# This file is intended to be ran using a roslaunch file
# The script continously records ROS bag files in 10 minute segments
# After an hour, the bag files begin to be overridden
# The rosbags are stored in ~/.ros/log

# exipt_script() is ran when the process recieves an interrupt or terminate request
exit_script() {
	echo "Exiting"
	# Reset the interrupt/terminate signals
	trap - INT TERM
	# Close the bag file we are currently recording (so that we don't end up with autorecord#.bag.active)
	rosnode kill /autorecord_bag
	# Kill this script
	kill -- -$$
}

# trap will run exit_script() when an interrupt (INT) or terminate (TERM) signal is recieved
# INT = SIGINT and TERM = SIGTERM
# The INT signal is recieved when you press "Ctrl-c"
trap exit_script INT TERM

while true
do
	for i in 1 2 3 4 5 6
	do
		echo 'Starting recording ~/.ros/log/autorecord'$i
		# Record all ROS topics for 10 minutes to a file named "autorecord#"
		# The node used to record the bag file is named "/autorecord_bag" (so it can be killed by name when this script is killed)
		rosbag record -a --duration=10m -O ~/.ros/log/autorecord$i __name:=autorecord_bag
	done
done
