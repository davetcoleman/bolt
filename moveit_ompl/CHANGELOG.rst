^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ompl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2016-10-25)
------------------
* Add PathGeometricPtr throughout codebase
  Move convertPath to model_based_state_space
* Add execution_interface
  Move remote_control to moveit_dashboard
  Param for hybrid cartesian dual arm combination factor
  Debug tolerances
* Cartesian planning working
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
* New settings
* Dual arm Cartesian planning working!
  Visualize smoothed raw path
  Optimize code
* Fix wrist bug for 6dof arm planning into 14dof two arms
  Remove vertex type from file saving
  Move combine state function into callback
  Split showing status into two functions
  Track down bug with cartesian planner having duplicate states
  New visualize function
* Add tests
  Fix travis for kinetic
  Start creating graph mirroring logic
* Smarter save interval logic
  Trigger API change
  Rename typedef TaskVertexMatrix
* New getAllIK() solutions
  Clang format
* Update rosinstall and remove jade version
  New visualization trigger method
  Removed old SamplingQueue class
  Generatlize sampler to be clearance or regular
  New benchmark
  Fix install in CMakeLists
* Consolidate settings
  Fix Baxter problems with Bolt
* Change stretch factor formula
  Add new parameters for testing changtes to BOLT
  Remove smoothQualityPathOriginal
  Add clearEdgesNearVertex to more places
  Optimize code more
* More settings for parameter sweeping and testing
  Optimize Bolt for faster astar solving
* Simplified level5 map
  Made Sweeper agregate mean and std dev
  Formatting
* Free more memory
* Remove SamplingQueue completely
  Improve memory leaks
  Simplify CandidateQueue
* Created PathSmoother class
  Copied SPARStwo into bolt_core/SPARS2
  Made SPARS2 use PathSmoother also
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
* Rename visualize smoothing parameter
  Fix segfaults
* Option to disable quality criteria
  Make debug functions shorter named
  Fix clearance bug
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
* Initial commit
* Contributors: Dave Coleman
