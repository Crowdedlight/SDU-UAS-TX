
# Whats what

The Arduino transmitter code is in => "python_controller"

Rosnodes is in catkin_ws => "remotecontrol_ws"

	The following nodes needs to be run to run the system
	"camera_controller" => PID controller for drone
	"marker_attitude" => Markerfinder and pose estimator
	"remote_control" => Interface from ros to Arduino transmitter

Matlab folder has the script for calculating graphs based on error.csv the controller outputs

