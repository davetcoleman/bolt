# Bolt MoveIt!

Description: Demonstrate dual arm manipulation using a combination of free space and Cartesian planning on Baxter

Features:

 - Integration of OMPL + MoveIt + Descartes-like features
 - Multi-modal task planning - have intermediate waypoint goals
 - Follow cartesian paths
 - Support for dual arms

## Usage

Start Rviz:

    roslaunch bolt_baxter baxter_visualize.launch

Disable Baxter collision checking for both arms so that they can move around each other closely (warning: this could cause harm to people):

    rostopic pub /robot/limb/left/suppress_collision_avoidance std_msgs/Empty -r 10
    rostopic pub /robot/limb/right/suppress_collision_avoidance std_msgs/Empty -r 10

Run the Baxter controller:

    rosrun baxter_interface joint_trajectory_action_server.py

Run example demo:

    roslaunch bolt_baxter bolt_baxter.launch

Note: the first time you run the program, it will discretize the configuration space in a brute-force manner. This will take many hours and at some point you should just stop it running any further. It will then save the database in the folder:

    ~/ros/ompl_storage/

If you want to skip this step you can download a pre-generated roadmap from [dav.ee/bolt_both_arms_0.500000_database.ompl](http://dav.ee/bolt_both_arms_0.500000_database.ompl) and place in that folder.

If you are prompted to press next to continue, find the "MoveItDashboard" panel in Rviz and click Next.

## Configuration

There are lots of settings that can easily be tweaked in the following file:

    bolt_baxter/config/config_baxter.yaml

In particular, pay attention to the ``visualize/`` configurations for more indepth view of what is going on.
