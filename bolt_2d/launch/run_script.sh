#/home/dave/ros/ws_moveit/devel/lib/ompl_experience_demos/thunder_spars_demo --verbose --runs 1

#~/ros/ws_moveit/devel/lib/ompl_experience_demos/thunder_spars_demo --verbose --runs 1 --image `rospack find ompl_experience_demos`/resources/hard2.ppm 

gdb -x /home/dave/ros/ws_moveit/src/ompl_experience_demos/launch/debug_settings.gdb --ex run --args /home/dave/ros/ws_moveit/devel/lib/ompl_experience_demos/thunder_spars_demo --noSaving --staticProblem
#--image ~/ros/ws_moveit/src/ompl_visual_tools/resources/simple.ppm 
