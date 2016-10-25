^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bolt_moveit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Improve documentation and install instructions
* Add approach to cartesian trajectory
  Visualize components of trajectory separate
  Repearedly execute trajectory
* Add PathGeometricPtr throughout codebase
  Move convertPath to model_based_state_space
* Convert ob::PathPtr to og::PathGeometricPtr
* Tweak settings for real performance
  connect_to_hardware var
  Improve cartesian discretization
* Execution working for Baxter
* Get execution working
* Add execution_interface
  Move remote_control to moveit_dashboard
  Param for hybrid cartesian dual arm combination factor
  Debug tolerances
* Cartesian planning working
* Fix initialization bug
  Made BOLT_FUNC use name\_
  Removed ExperienceSetup in favor or simply SimpleSetup
  Changed API for TaskGraph to clarify meaning
* Clarify modelSI vs compoundSI
  Cleanup code
* Add collision checking to descartes planner
  Improve how descartes planner creates points
  New method for combining two arms of states together
  Change debug output
* Fix populateNNThread() bug
  Fix BoltPlanner planning
  Collision checking cartesian paths
* Continue to fix BoltPlanner
  Remove getLevel setLevel from OMPL
  Fix usage of collision_state\_ in searching in TaskGraph
* Fix disjoint sets
* clang-format all
* Made BOLT_ERROR not have flag
* Improve BoltPlanner for compound states
  New distance() functions
  Make DefaultStateSampler a template
* Use CompoundStates with DiscreteState to save memory
  New createCompoundState() function
  Improve tests
  Make weights use floats
  Cleanup BoostGraphHeaders
* Switch SparseGraph to use structs instead of propery maps for Boost Graph
  Speedup file loading
  Optimize memory usage
  Removed unused code
* Before portland progress
* Dual arm Cartesian planning working!
  Visualize smoothed raw path
  Optimize code
* Rename launch file
* Fix wrist bug for 6dof arm planning into 14dof two arms
  Remove vertex type from file saving
  Move combine state function into callback
  Split showing status into two functions
  Track down bug with cartesian planner having duplicate states
  New visualize function
* Experimented with new astar test for mirroring graph - did not help
  Small optimizations for mirroring
* Remove types from vertices and edges to save space
  Remove disjoint sets from TaskGraph
* Fix segfault with cartesian planning
* Implement dual arm cartesian planning (untested)
  Switch to ArmData datastructure
* Support for multiple end effectors
  Speed up SparseGraph when mirroring
* Cleanup mirror class
  Allow for two end effectors
* Split mirroring functionality into new class
* Improve graph mirroring
  Check arm mirror validity
  More validity checkers
* VizWindow supports changing SpaceInformation during runtime
  Copy graph mirror logic
* Fix segfaults
* Fix test
* Fix travis
* Writing moveit test
* Add tests
  Fix travis for kinetic
  Start creating graph mirroring logic
* Allow path() to specify two colors
* Allow mono-level planning to occur in TaskGraph
* Fix replanning bug with clear()
  Speed up astar for taskgraph
  Make IK solving at tip of fingers, not gripper
* Fix discretization for Baxter
  New rosparam settings
  Rename cart_path_planner functions
  Better console debugging
  Better visualization options
* Smarter save interval logic
  Trigger API change
  Rename typedef TaskVertexMatrix
* Fix disjoint sets visualization
  Remove descartes dependency
  Debug cartesian planning
* Replace ur5_robot_model isValidMove() with new version in moveit::core::RobotState
  Fix cartesian planning
* New getAllIK() solutions
  Clang format
* Disable quality option for faster graph building
* Update rosinstall and remove jade version
  New visualization trigger method
  Removed old SamplingQueue class
  Generatlize sampler to be clearance or regular
  New benchmark
  Fix install in CMakeLists
* Moved SPAR2 files to main folder
  Fix projection_viz_window assertion for 7 dof
* Collision checking
* Consolidate settings
  Fix Baxter problems with Bolt
* Renamed more things from Hilgendorf to MoveIt
* Added bolt_moveit
* Renamed to bolt_hilgendorf
* Initial commit
* Contributors: Dave Coleman
