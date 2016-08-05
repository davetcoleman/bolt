/*********************************************************************
 * Software License Agreement (SBD License)
 *
 *  Copyright (c) 2016, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman */

// OMPL
#include <ompl/base/PlannerStatus.h>
#include <ompl/util/Console.h>

// Bolt
#include <ompl/tools/bolt/Bolt.h>
#include <ompl/tools/bolt/SparseGenerator.h>
#include <ompl/tools/bolt/SparseCriteria.h>
#include <ompl/base/samplers/MinimumClearanceValidStateSampler.h>

//#include <ompl/geometric/planners/rrt/RRTConnect.h>

namespace og = ompl::geometric;
namespace ob = ompl::base;
namespace ot = ompl::tools;

namespace ompl
{
namespace tools
{
namespace bolt
{
Bolt::Bolt(const base::SpaceInformationPtr &si) : ExperienceSetup(si)
{
  initialize();
}

Bolt::Bolt(const base::StateSpacePtr &space) : ExperienceSetup(space)
{
  initialize();
}

void Bolt::initialize()
{
  OMPL_INFORM("Initializing Bolt Framework");

  // Initalize visualizer class
  visual_.reset(new Visualizer());

  recallEnabled_ = true;
  scratchEnabled_ = true;
  filePath_ = "unloaded";

  // Load the sparse graph datastructure
  sparseGraph_.reset(new SparseGraph(si_, visual_));

  // Load criteria used to determine if samples are saved or rejected
  sparseCriteria_.reset(new SparseCriteria(sparseGraph_));

  // Load the generator of sparse vertices and edges
  sparseGenerator_.reset(new SparseGenerator(sparseGraph_));

  // Give the sparse graph reference to the criteria, because sometimes it needs data from there
  sparseGraph_->setSparseCriteria(sparseCriteria_);
  sparseGenerator_->setSparseCriteria(sparseCriteria_);

  // Load the task graph used for combining multiple layers of sparse graph
  taskGraph_.reset(new TaskGraph(sparseGraph_));

  // Load the Retrieve repair database. We do it here so that setRepairPlanner() works
  boltPlanner_ = BoltPlannerPtr(new BoltPlanner(si_, taskGraph_, visual_));

  std::size_t numThreads = boost::thread::hardware_concurrency();
  OMPL_INFORM("Bolt Framework initialized using %u threads", numThreads);
}

void Bolt::setup()
{
  std::size_t indent = 0;

  if (!configured_ || !si_->isSetup() || !boltPlanner_->isSetup())
  {
    // Setup Space Information if we haven't already done so
    if (!si_->isSetup())
      si_->setup();

    // Setup planning from experience planner
    boltPlanner_->setProblemDefinition(pdef_);

    if (!boltPlanner_->isSetup())
      boltPlanner_->setup();

    // Setup SPARS
    sparseGraph_->setup();
    sparseCriteria_->setup(indent);
    sparseGenerator_->setup(indent);
    taskGraph_->setup();

    // Set the configured flag
    configured_ = true;
  }
}

void Bolt::clear(void)
{
  if (boltPlanner_)
    boltPlanner_->clear();
  if (pdef_)
    pdef_->clearSolutionPaths();
}

void Bolt::setPlannerAllocator(const base::PlannerAllocator &pa)
{
  pa_ = pa;
  // note: the boltPlanner_ never uses the allocator so does not need to be reset
  configured_ = false;
}

base::PlannerStatus Bolt::solve(const base::PlannerTerminationCondition &ptc)
{
  // Setup again in case it has not been done yet
  setup();

  lastStatus_ = base::PlannerStatus::UNKNOWN;
  time::point start = time::now();

  // Warn if there are queued paths that have not been added to the experience database
  OMPL_INFORM("Num solved paths uninserted into the experience database in the post-proccessing queue: %u",
              queuedSolutionPaths_.size());

  // SOLVE
  lastStatus_ = boltPlanner_->solve(ptc);

  // Task time
  planTime_ = time::seconds(time::now() - start);

  // Do logging
  logResults();

  return lastStatus_;
}

void Bolt::visualize()
{
  // Optionally visualize raw trajectory
  if (visualizeRawTrajectory_)
  {
    std::shared_ptr<geometric::PathGeometric> originalPath = boltPlanner_->getOriginalSolutionPath();

    // Make the chosen path a different color and thickness
    visual_->viz5()->path(originalPath.get(), tools::MEDIUM, tools::BLACK);
    visual_->viz5()->trigger();

    // Don't show raw trajectory twice in larger dimensions
    if (si_->getStateSpace()->getDimension() == 3)
    {
      visual_->viz6()->path(originalPath.get(), tools::MEDIUM, tools::BLACK);
      visual_->viz6()->trigger();
    }
  }

  geometric::PathGeometric *solutionPath = static_cast<geometric::PathGeometric *>(pdef_->getSolutionPath().get());

  // Make a copy so that we can interpolate it
  geometric::PathGeometric solutionPathCopy(*solutionPath);
  solutionPathCopy.interpolate();

  // Show smoothed & interpolated path
  if (visualizeSmoothTrajectory_)
  {
    visual_->viz6()->path(&solutionPathCopy, tools::LARGE, tools::PURPLE);
    visual_->viz6()->trigger();
  }

  // Show robot animated robot
  if (visualizeRobotTrajectory_)
  {
    std::size_t indent = 0;
    BOLT_DEBUG(indent, true, "Blocking while visualizing solution path");
    visual_->viz6()->path(&solutionPathCopy, tools::ROBOT, tools::DEFAULT);
  }
}

bool Bolt::checkOptimalityGuarantees(std::size_t indent)
{
  geometric::PathGeometric *rawPath = boltPlanner_->getOriginalSolutionPath().get();
  geometric::PathGeometric *smoothedPath = static_cast<geometric::PathGeometric *>(pdef_->getSolutionPath().get());

  double optimalLength = smoothedPath->length();
  double sparseLength = rawPath->length();
  double theoryLength = sparseCriteria_->getStretchFactor() * optimalLength + 4 * sparseCriteria_->getSparseDelta();
  double percentOfMaxAllows = sparseLength / theoryLength * 100.0;

  BOLT_DEBUG(indent, 1, "-----------------------------------------");
  BOLT_DEBUG(indent, 1, "Checking Asymptotic Optimality Guarantees");
  BOLT_DEBUG(indent + 2, 1, "Raw Path Length:         " << sparseLength);
  BOLT_DEBUG(indent + 2, 1, "Smoothed Path Length:    " << optimalLength);
  BOLT_DEBUG(indent + 2, 1, "Theoretical Path Length: " << theoryLength);
  BOLT_DEBUG(indent + 2, 1, "Stretch Factor t:        " << sparseCriteria_->getStretchFactor());
  BOLT_DEBUG(indent + 2, 1, "Sparse Delta:            " << sparseCriteria_->getSparseDelta());

  if (sparseLength >= theoryLength)
  {
    BOLT_ERROR(indent + 2, 1, "Asymptotic optimality guarantee VIOLATED");
    return false;
  }
  else
    BOLT_GREEN_DEBUG(indent + 2, 1, "Asymptotic optimality guarantee maintained");
  BOLT_WARN(indent + 2, 1, "Percent of max allowed:  " << percentOfMaxAllows << " %");
  BOLT_DEBUG(indent, 1, "-----------------------------------------");

  // visual_->waitForUserFeedback("review results");

  return true;
}

void Bolt::logResults()
{
  // Create log
  ExperienceLog log;
  log.planningTime = planTime_;

  // Record stats
  stats_.totalPlanningTime_ += planTime_;  // used for averaging
  stats_.numProblems_++;                   // used for averaging

  switch (static_cast<ompl::base::PlannerStatus::StatusType>(lastStatus_))
  {
    case base::PlannerStatus::TIMEOUT:
      stats_.numSolutionsTimedout_++;
      OMPL_ERROR("Bolt::solve(): TIMEOUT - No solution found after %f seconds", planTime_);
      // Logging
      log.planner = "neither_planner";
      log.result = "timedout";
      log.isSaved = "not_saved";
      break;
    case base::PlannerStatus::ABORT:
      stats_.numSolutionsTimedout_++;
      OMPL_ERROR("Bolt::solve(): ABORT - No solution found after %f seconds", planTime_);
      // Logging
      log.planner = "neither_planner";
      log.result = "abort";
      log.isSaved = "not_saved";
      break;
    case base::PlannerStatus::APPROXIMATE_SOLUTION:
      OMPL_ERROR("Bolt::solve(): Approximate - should not happen!");
      exit(-1);
      break;
    case base::PlannerStatus::EXACT_SOLUTION:
    {
      og::PathGeometric solutionPath = og::SimpleSetup::getSolutionPath();  // copied so that it is non-const

      std::cout << ANSI_COLOR_BLUE;
      std::cout << "Bolt Finished - solution found in " << planTime_ << " seconds with " << solutionPath.getStateCount()
                << " states" << std::endl;
      std::cout << ANSI_COLOR_RESET;

      // Show in Rviz
      visualize();

      // Error check for repeated states
      if (!checkRepeatedStates(solutionPath))
        exit(-1);

      // Check optimality
      // if (!checkOptimalityGuarantees())
      // exit(-1);

      // Stats
      stats_.numSolutionsFromRecall_++;

      // Make sure solution has at least 2 states
      if (solutionPath.getStateCount() < 2)
      {
        OMPL_INFORM("NOT saving to database because solution is less than 2 states long");
        stats_.numSolutionsTooShort_++;

        // Logging
        log.isSaved = "less_2_states";
        log.tooShort = true;
      }
      else
      {
        // Queue the solution path for future insertion into experience database (post-processing)
        queuedSolutionPaths_.push_back(solutionPath);
      }
    }
    break;
    default:
      OMPL_ERROR("Unknown status type: %u", lastStatus_);
      stats_.numSolutionsFailed_++;
      // Logging
      log.planner = "neither_planner";
      log.result = "failed";
      log.isSaved = "not_saved";
  }

  // Final log data
  // log.insertion_time = insertionTime; TODO fix this
  log.numVertices = sparseGraph_->getNumVertices();
  log.numEdges = sparseGraph_->getNumEdges();
  log.numConnectedComponents = 0;

  // Flush the log to buffer
  convertLogToString(log);
}

bool Bolt::checkRepeatedStates(const og::PathGeometric &path)
{
  for (std::size_t i = 1; i < path.getStateCount(); ++i)
  {
    if (si_->getStateSpace()->equalStates(path.getState(i - 1), path.getState(i)))
    {
      OMPL_ERROR("Duplicate state found on trajectory at %u out of %u", i, path.getStateCount());

      visual_->viz6()->state(path.getState(i), tools::ROBOT, tools::RED, 0);
      return false;
    }
  }
  return true;
}

base::PlannerStatus Bolt::solve(double time)
{
  ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(time);
  return solve(ptc);
}

bool Bolt::setFilePath(const std::string &filePath)
{
  sparseGraph_->setFilePath(filePath + ".ompl");
  sparseGraph_->getSparseStorage()->setLoggingPath(filePath + ".logging");
  benchmarkFilePath_ = filePath + ".benchmark";
  return true;
}

bool Bolt::save()
{
  // setup(); // ensure the db has been loaded to the Experience DB
  return sparseGraph_->save();
}

bool Bolt::saveIfChanged()
{
  // setup(); // ensure the db has been loaded to the Experience DB
  return sparseGraph_->saveIfChanged();
}

void Bolt::printResultsInfo(std::ostream &out) const
{
  for (std::size_t i = 0; i < pdef_->getSolutionCount(); ++i)
  {
    out << "Solution " << i << "\t | Length: " << pdef_->getSolutions()[i].length_
        << "\t | Approximate: " << (pdef_->getSolutions()[i].approximate_ ? "true" : "false")
        << "\t | Planner: " << pdef_->getSolutions()[i].plannerName_ << std::endl;
  }
}

bool Bolt::load()
{
  std::size_t indent = 0;

  // Load from file
  if (!sparseGraph_->isEmpty())
  {
    BOLT_WARN(indent, 1, "Database already loaded, vertices: " << sparseGraph_->getNumVertices()
                                                               << ", edges: " << sparseGraph_->getNumEdges()
                                                               << ", queryV: " << sparseGraph_->getNumQueryVertices());
    return false;
  }

  if (!sparseGraph_->load())  // load from file
  {
    return false;
  }

  return true;
}

void Bolt::print(std::ostream &out) const
{
  if (si_)
  {
    si_->printProperties(out);
    si_->printSettings(out);
  }
  if (boltPlanner_)
  {
    boltPlanner_->printProperties(out);
    boltPlanner_->printSettings(out);
  }
  if (pdef_)
    pdef_->print(out);
}

void Bolt::printLogs(std::ostream &out) const
{
  double vertPercent = sparseGraph_->getNumVertices() / double(sparseGraph_->getNumVertices()) * 100.0;
  double edgePercent = sparseGraph_->getNumEdges() / double(sparseGraph_->getNumEdges()) * 100.0;
  double solvedPercent = stats_.numSolutionsFromRecall_ / static_cast<double>(stats_.numProblems_) * 100.0;
  if (!recallEnabled_)
    out << "Scratch Task Logging Results (inside Bolt Framework)" << std::endl;
  else
    out << "Bolt Framework Logging Results" << std::endl;
  out << "  Solutions Attempted:           " << stats_.numProblems_ << std::endl;
  out << "    Solved:                      " << stats_.numSolutionsFromRecall_ << " (" << solvedPercent << "%)\n";
  out << "    Failed:                      " << stats_.numSolutionsFailed_ << std::endl;
  out << "    Timedout:                    " << stats_.numSolutionsTimedout_ << std::endl;
  out << "    Approximate:                 " << stats_.numSolutionsApproximate_ << std::endl;
  out << "  SparseGraph                       " << std::endl;
  out << "    Vertices:                    " << sparseGraph_->getNumVertices() << " (" << vertPercent << "%)"
      << std::endl;
  out << "    Edges:                       " << sparseGraph_->getNumEdges() << " (" << edgePercent << "%)" << std::endl;
  out << "    Disjoint Samples Added:      " << sparseGenerator_->getNumRandSamplesAdded() << std::endl;
  out << "    Sparse Delta:                " << sparseCriteria_->getSparseDelta() << std::endl;
  out << "  Average planning time:         " << stats_.getAveragePlanningTime() << " seconds" << std::endl;
  out << "  Average insertion time:        " << stats_.getAverageInsertionTime() << " seconds" << std::endl;
  out << std::endl;
}

std::size_t Bolt::getExperiencesCount() const
{
  return sparseGraph_->getNumVertices();
}

void Bolt::getAllPlannerDatas(std::vector<ob::PlannerDataPtr> &plannerDatas) const
{
  // sparseGraph_->getAllPlannerDatas(plannerDatas);
}

void Bolt::convertPlannerData(const ob::PlannerDataPtr plannerData, og::PathGeometric &path)
{
  // Convert the planner data verticies into a vector of states
  for (std::size_t i = 0; i < plannerData->numVertices(); ++i)
    path.append(plannerData->getVertex(i).getState());
}

SparseGraphPtr Bolt::getSparseGraph()
{
  return sparseGraph_;
}

TaskGraphPtr Bolt::getTaskGraph()
{
  return taskGraph_;
}

SparseCriteriaPtr Bolt::getSparseCriteria()
{
  return sparseCriteria_;
}

bool Bolt::doPostProcessing()
{
  OMPL_INFORM("Performing post-processing for %i queued solution paths", queuedSolutionPaths_.size());
  OMPL_INFORM("TODO post-processing");

  return true;
}

void Bolt::benchmarkRandValidSampling()
{
  std::cout << "-------------------------------------------------------" << std::endl;
  OMPL_INFORM("Running system performance benchmark");
  base::State *candidateState = si_->getStateSpace()->allocState();

  base::StateSamplerPtr sampler;
  sampler = si_->allocStateSampler();

  const std::size_t benchmarkRuns = 10000;
  std::size_t debugIncrement = benchmarkRuns / 10;
  std::size_t validCount = 0;

  // Benchmark runtime
  double totalDurationValid = 0;
  time::point startTimeValid;
  double totalDurationSampler = 0;
  time::point startTimeSampler;
  for (std::size_t i = 0; i < benchmarkRuns; ++i)
  {
    startTimeSampler = time::now();
    sampler->sampleUniform(candidateState);

    startTimeValid = time::now();
    totalDurationSampler += time::seconds(startTimeValid - startTimeSampler);

    validCount += si_->isValid(candidateState);
    totalDurationValid += time::seconds(time::now() - startTimeValid);

    if (i % debugIncrement == 0)
      std::cout << "Progress: " << i / double(benchmarkRuns) * 100.0 << "%" << std::endl;
  }
  // Benchmark runtime
  OMPL_INFORM("  isValid() took %f seconds (%f per run)", totalDurationValid, totalDurationValid / benchmarkRuns);
  OMPL_INFORM("  sampleUniform() took %f seconds (%f per run)", totalDurationSampler,
              totalDurationSampler / benchmarkRuns);
  OMPL_INFORM("  Percent valid: %f", validCount / double(benchmarkRuns) * 100);
  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << std::endl;
}

void Bolt::benchmarkVisualizeSampling()
{
  std::cout << "-------------------------------------------------------" << std::endl;
  OMPL_INFORM("Running system performance benchmark");

  base::MinimumClearanceValidStateSamplerPtr clearanceSampler(new ob::MinimumClearanceValidStateSampler(si_.get()));
  clearanceSampler->setMinimumObstacleClearance(sparseGraph_->getObstacleClearance());

  base::StateSamplerPtr sampler;
  sampler = si_->allocStateSampler();

  const std::size_t benchmarkRuns = 100000;
  std::size_t debugIncrement = std::max(benchmarkRuns, std::size_t(benchmarkRuns / 100.0));

  // Pre-allocate state
  std::vector<ompl::base::State*> stateMemory(debugIncrement);

  for (base::State*& state : stateMemory)
    state = si_->getStateSpace()->allocState();

  std::vector<const ompl::base::State*> states;
  states.reserve(debugIncrement);
  std::vector<ot::VizColors> colors;
  colors.reserve(debugIncrement);

  // Allow time to reset image
  visual_->viz1()->trigger();
  usleep(0.1*1000000);

  // Benchmark runtime
  for (std::size_t i = 0; i < benchmarkRuns; ++i)
  {
    base::State *candidateState = stateMemory[i % debugIncrement];
    clearanceSampler->sample(candidateState);

    states.push_back(candidateState);
    colors.push_back(tools::GREEN);

    if ((i+1) % debugIncrement == 0 || i == benchmarkRuns - 1)
    {
      visual_->viz1()->states(states, colors, tools::SMALL);
      visual_->viz1()->trigger();
      usleep(0.1*1000000);

      states.clear();
      colors.clear();
      if (visual_->viz1()->shutdownRequested())
        break;
    }
  }

  for (base::State* state : stateMemory)
    si_->freeState(state);

  // Benchmark runtime
  OMPL_INFORM("Done");
  exit(0);
  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << std::endl;
}

void Bolt::benchmarkSparseGraphGeneration()
{
  std::cout << "-------------------------------------------------------" << std::endl;
  OMPL_INFORM("Running graph generation benchmark");
  time::point startTime = time::now();  // Benchmark

  // Create graph
  sparseGenerator_->createSPARS();

  double time = time::seconds(time::now() - startTime);
  OMPL_INFORM("Graph generation took %f seconds", time);  // Benchmark

  std::ofstream loggingFile;  // open to append
  // loggingFile.open(benchmarkFilePath_.c_str(), std::ios::out);  // no append
  loggingFile.open(benchmarkFilePath_.c_str(), std::ios::app);  // append
  loggingFile << time << ", " << sparseGraph_->getNumEdges() << ", " << sparseGraph_->getNumVertices() << std::endl;
  loggingFile.close();

  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << std::endl;
}

}  // namespace bolt
}  // namespace tools
}  // namespace ompl
