^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bolt_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Converted TaskGraph to use struts instead of propery maps
* Switch SparseGraph to use structs instead of propery maps for Boost Graph
  Speedup file loading
  Optimize memory usage
  Removed unused code
* Before portland progress
* Dual arm Cartesian planning working!
  Visualize smoothed raw path
  Optimize code
* Fix wrist bug for 6dof arm planning into 14dof two arms
  Remove vertex type from file saving
  Move combine state function into callback
  Split showing status into two functions
  Track down bug with cartesian planner having duplicate states
  New visualize function
* Experimented with new astar test for mirroring graph - did not help
  Small optimizations for mirroring
* Rename graphs in SparseMirror
* Remove types from vertices and edges to save space
  Remove disjoint sets from TaskGraph
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
* Mirror graph logic
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
* Last minute before paper submission
* Change stretch factor formula
  Add new parameters for testing changtes to BOLT
  Remove smoothQualityPathOriginal
  Add clearEdgesNearVertex to more places
  Optimize code more
* More settings for parameter sweeping and testing
  Optimize Bolt for faster astar solving
* Set clearance for sparse2
* Made level5 map a little harder
  Created SparseFormula to share the same t-stretch factor
  Made SPARS2 use new logging method
* Simplified level5 map
  Made Sweeper agregate mean and std dev
  Formatting
* Removed checkRemoveCloseVertices() functionality
* Allow more than one node to run at a time
  Improve feedback
  Make candidate queue one thread
* Improve CandidateQueue memory leak
* Fix major memory leaks
  Rename ExperienceDemos class
  Add lots of std::move() - not sure if necessary
* Free more memory
* Remove SamplingQueue completely
  Improve memory leaks
  Simplify CandidateQueue
* Reduce visualizations during benchmarking
* Created PathSmoother class
  Copied SPARStwo into bolt_core/SPARS2
  Made SPARS2 use PathSmoother also
* Cleanup
* Start testing SPARS2
* Added 5 levels of trial images
  Created function for sweeping different trial images
  Changed interface criteria to not care about closest vertices
  Better logging data functionality
* Add new parameters for CandidateQueue
  Turn back on CandidateQueue
  Fix CMakeLists
  Better user feedback
  Move initialization of threading
  Reduce console output
* Make graph verification optional
  Rearrange display windows ordering
  Click and add vertex interactive feature for Rviz
  Spin ability for OMPL visualize
  Better path smoothing error handling
  Switch to multi-threaded graph creation
  Only verify graph when in debug mode
  addSample() helper function
* More debug tools
* Began adding test but gave up
* Passes tests
* Improve debugging and testing
* Fix clearance bug
* Fixing bugs
* Rename visualize smoothing parameter
  Fix segfaults
* Fix memory leaks
* Fixes for kinetic
* Cropped
* Fix image path
* Added diagrams
* Documented sparse graph
* Update READMEs and fix rosinstall
* Multi spars graph generation testing
* Ability to generate spars graph multiple times
  Clearing functions for resetting spars
  Improved visibility area visualization
  cons InterfaceData functions
  Use checkPathLength for Quality criteria
  inline various functions
  Add inline documentation
* Option to disable quality criteria
  Make debug functions shorter named
  Fix clearance bug
* Move benchmarks to SparseGenerator
  Replace typedef
  Cleanup Bolt
* Removed unused SampleQueue
  Cleanup clearance
  Improve sparseGenerator output
* Good smoother
* Visualizations
* More cleanup
* Removed ompl_visual_tools
  Fixed installation of packages
* Clang format on entire project
* Renamed launch files
  Fix save bug
  Added new throw assert method
  Switched to BOLT_ASSERT
  Faster astar search function
  New checkSparseGraphOptimality function
  New smoothMax()
* Cleaning up
* bolt_core rename
* Initial commit
* Contributors: Dave Coleman
