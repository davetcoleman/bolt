#/home/dave/ros/ws_moveit/devel/lib/bolt_2d/thunder_spars_demo --verbose --runs 1

#~/ros/ws_moveit/devel/lib/bolt_2d/thunder_spars_demo --verbose --runs 1 --image `rospack find bolt_2d`/resources/hard2.ppm 

gdb -x /home/dave/ros/ws_moveit/src/bolt_2d/launch/debug_settings.gdb --ex run --args /home/dave/ros/ws_moveit/devel/lib/bolt_2d/thunder_spars_demo --noSaving --staticProblem
#--image ~/ros/ws_moveit/src/ompl_visual_tools/resources/simple.ppm 
