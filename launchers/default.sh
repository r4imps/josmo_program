#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
#roslaunch my_package multiple_nodes.launch veh:=$VEHICLE_NAME
dt-exec roslaunch imu_driver imu_node.launch veh:=$VEHICLE_NAME
#dt-exec rosrun my_package my_publisher_node.py 
#dt-exec rosrun my_package my_subscriber_node.py 
dt-exec rosrun my_package ob_detect.py
#dt-exec rosrun my_package line_value.py
dt-exec rosrun my_package PID_Controller.py
dt-exec rosrun my_package josmo_stronk.py
#dt-exec rosrun my_package LED_Emitter.py
#dt-exec rosrun my_package imu.py
#dt-exec rosrun my_package odometry.py


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
