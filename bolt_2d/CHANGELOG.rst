^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bolt_2d
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2016-10-25)
------------------
* Add execution_interface
  Move remote_control to moveit_visual_tools
  Param for hybrid cartesian dual arm combination factor
  Debug tolerances
* Fix initialization bug
  Made BOLT_FUNC use name\_
  Removed ExperienceSetup in favor or simply SimpleSetup
  Changed API for TaskGraph to clarify meaning
* Fix populateNNThread() bug
  Fix BoltPlanner planning
  Collision checking cartesian paths
* Continue to fix BoltPlanner
  Remove getLevel setLevel from OMPL
  Fix usage of collision_state\_ in searching in TaskGraph
* clang-format all
* VizWindow supports changing SpaceInformation during runtime
  Copy graph mirror logic
* Writing moveit test
* Add tests
  Fix travis for kinetic
  Start creating graph mirroring logic
* Allow path() to specify two colors
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
* Moved SPAR2 files to main folder
  Fix projection_viz_window assertion for 7 dof
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
* screenshot
* Document and cleanup
* Added screenshot and docs
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
* working towards removing ompl_visual_tools
  removed cost capabilities from 2d world
  switched to using OMPL's ppm
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
* Renamed everything to bolt_2d
* Initial commit
* Contributors: Dave Coleman
