/**
 * \brief Add a new solution path to our database. Des not actually save to file so
 *        experience will be lost if save() is not called
 * \param new path - must be non-const because will be interpolated
 * \return true on success
 */
bool postProcessPath(geometric::PathGeometric &solutionPath);

/** \brief Helper function for postProcessPath */
bool postProcessPathWithNeighbors(geometric::PathGeometric &solutionPath,
                                  const std::vector<TaskVertex> &visibleNeighborhood, bool recurseVerbose,
                                  std::vector<TaskVertex> &roadmapPath);

/** \brief Update edge weights of dense graph based on this a newly created path */
bool updateEdgeWeights(const std::vector<TaskVertex> &roadmapPath);

/**
 * \brief Call itself recursively for each point in the trajectory, looking for vertices on the graph to connect to
 * \param inputPath - smoothed trajectory that we want to now convert into a 'snapped' trajectory
 * \param roadmapPath - resulting path that is 'snapped' onto the pre-existing roadmap
 * \param currVertexIndex - index in inputPath (smoothed path) that we are trying to connect to
 * \param prevGraphVertex - vertex we are trying to connect to
 * \param allValid - flag to determine if any of the edges were never found to be valid
 * \param verbose - whether to show lots of debug output to console
 * \return true on success
 */
bool recurseSnapWaypoints(ompl::geometric::PathGeometric &inputPath, std::vector<TaskVertex> &roadmapPath,
                          std::size_t currVertexIndex, const TaskVertex &prevGraphVertex, bool &allValid, bool verbose);

/** \brief Keep graph evenly weighted */
void normalizeGraphEdgeWeights();


/** \brief Check that all states are the correct type */
void checkStateType();

/**
 * \brief Sometimes the dense graph's discretization is not enough, so we have the ability to manually add
 *        samples and connect to the graph
 * \param denseV - a newly created vertex that needs to be connected to the dense graph
 *                 the vertex should cover a currently un-visible region of the configuration space
 */
void connectNewVertex(TaskVertex denseV);

void removeInvalidVertices();


bool TaskGraph::postProcessPath(og::PathGeometric &solutionPath)
{
  // Prevent inserting into database
  if (!savingEnabled_)
  {
    OMPL_WARN("TaskGraph: Saving is disabled so not adding path");
    return false;
  }

  if (visualizeSnapPath_)  // Clear old path
  {
    visual_->viz5()->deleteAllMarkers();
    visual_->viz4()->deleteAllMarkers();
  }

  // Get starting state
  base::State *currentPathState = solutionPath.getStates()[0];

  // Get neighbors
  std::vector<TaskVertex> graphNeighborhood;
  std::vector<TaskVertex> visibleNeighborhood;
  std::size_t coutIndent = 0;
  const std::size_t numThreads = 0;
  findGraphNeighbors(currentPathState, graphNeighborhood, visibleNeighborhood, sparseGraph_->sparseDelta_, numThreads,
                     coutIndent);

  std::vector<TaskVertex> roadmapPath;

  // Run in non-debug mode
  bool recurseVerbose = snapPathVerbose_;
  if (!postProcessPathWithNeighbors(solutionPath, visibleNeighborhood, recurseVerbose, roadmapPath))
  {
    OMPL_ERROR("Could not find snap waypoint path. Running again in debug");
    std::cout << "-------------------------------------------------------" << std::endl;

    // Run in debug mode
    recurseVerbose = true;
    visualizeSnapPath_ = true;
    roadmapPath.clear();
    postProcessPathWithNeighbors(solutionPath, visibleNeighborhood, recurseVerbose, roadmapPath);

    // temp
    std::cout << "exiting for debug " << std::endl;
    exit(-1);
  }

  // Error check
  if (roadmapPath.size() < 2)
  {
    OMPL_WARN("Snapped path waypoint count is too short, only contains %u waypoints", roadmapPath.size());
    if (roadmapPath.empty())
    {
      OMPL_ERROR("Trajectory completely empty!");
      exit(-1);
    }
    // It is possible to have a path of [actualStart, middlePoint, actualGoal], in which case we can't save any
    // experience from it
  }

  if (roadmapPath.size() > 100)
    OMPL_WARN("Roadmap size is %u", roadmapPath.size());

  if (snapPathVerbose_)
    std::cout << "Finished recurseSnapWaypoints(), now updating edge weights in Dense graph " << std::endl;

  // Update edge weights based on this newly created path
  updateEdgeWeights(roadmapPath);

  return true;
}

bool TaskGraph::postProcessPathWithNeighbors(og::PathGeometric &solutionPath,
                                             const std::vector<TaskVertex> &visibleNeighborhood, bool recurseVerbose,
                                             std::vector<TaskVertex> &roadmapPath)
{
  std::size_t currVertexIndex = 1;

  // Remember if any connections failed
  bool allValid = true;

  for (std::size_t i = 0; i < visibleNeighborhood.size(); ++i)
  {
    if (recurseVerbose)
      std::cout << "Attempting to start with neighbor " << i << std::endl;
    TaskVertex prevGraphVertex = visibleNeighborhood[i];

    if (visualizeSnapPath_)  // Add first state
    {
      visual_->viz5()->state(stateProperty_[prevGraphVertex], tools::SMALL, tools::GREEN, 1);
    }

    // Add this start state
    roadmapPath.push_back(prevGraphVertex);

    // Start recursive function
    allValid = true;
    if (!recurseSnapWaypoints(solutionPath, roadmapPath, currVertexIndex, prevGraphVertex, allValid, recurseVerbose))
    {
      std::cout << "Failed to find path with starting state neighbor " << i << std::endl;
    }
    else
    {
      break;  // sucess
    }

    if (visualizeSnapPath_)  // Visualize
    {
      visual_->viz5()->trigger();
      usleep(visualizeSnapPathSpeed_ * 1000000);
    }
  }

  return allValid;
}

bool TaskGraph::updateEdgeWeights(const std::vector<TaskVertex> &roadmapPath)
{
  for (std::size_t vertexID = 1; vertexID < roadmapPath.size(); ++vertexID)
  {
    std::pair<TaskEdge, bool> edgeResult = boost::edge(roadmapPath[vertexID - 1], roadmapPath[vertexID], g_);
    TaskEdge &edge = edgeResult.first;

    // Error check
    if (!edgeResult.second)
    {
      std::cout << std::string(2, ' ') << "WARNING: No edge found on snapped path at index " << vertexID
                << ", unable to save popularity of this edge. perhaps path needs interpolation first" << std::endl;

      if (visualizeSnapPath_)  // Visualize
      {
        const double cost = 100;  // red
        visual_->viz4()->edge(stateProperty_[roadmapPath[vertexID - 1]], stateProperty_[roadmapPath[vertexID]], cost);
        visual_->viz4()->trigger();
        usleep(visualizeSnapPathSpeed_ * 1000000);
      }
      std::cout << "shutting down out of curiosity " << std::endl;
      exit(-1);
    }
    else
    {
      // reduce cost of this edge because it was just used (increase popularity)
      // Note: 100 is an *unpopular* edge, and 0 is a super highway
      if (snapPathVerbose_)
      {
        std::cout << "Edge weight for vertex " << vertexID << " of edge " << edge << std::endl;
        std::cout << "    old: " << edgeWeightProperty_[edge];
      }
      edgeWeightProperty_[edge] = std::max(edgeWeightProperty_[edge] - POPULARITY_WEIGHT_REDUCTION, 0.0);
      if (snapPathVerbose_)
        std::cout << " new: " << edgeWeightProperty_[edge] << std::endl;

      if (visualizeSnapPath_)  // Visualize
      {
        visual_->viz5()->edge(stateProperty_[roadmapPath[vertexID - 1]], stateProperty_[roadmapPath[vertexID]], 100);
      }
    }
  }

  if (visualizeSnapPath_)  // Visualize
  {
    visual_->viz5()->trigger();
    usleep(visualizeSnapPathSpeed_ * 1000000);
  }

  return true;
}

bool TaskGraph::recurseSnapWaypoints(og::PathGeometric &inputPath, std::vector<TaskVertex> &roadmapPath,
                                     std::size_t currVertexIndex, const TaskVertex &prevGraphVertex, bool &allValid,
                                     bool verbose)
{
  if (verbose)
    std::cout << std::string(currVertexIndex, ' ') << "recurseSnapWaypoints() -------" << std::endl;

  // Find multiple nearby nodes on the graph
  std::vector<TaskVertex> graphNeighborhood;

  // Get the next state
  base::State *currentPathState = inputPath.getState(currVertexIndex);

  // Find multiple nearby nodes on the graph
  std::size_t threadID = 0;
  stateProperty_[queryVertices_[threadID]] = currentPathState;
  std::size_t findNearestKNeighbors = 10;
  const std::size_t numSameVerticiesFound = 1;  // add 1 to the end because the NN tree always returns itself
  nn_->nearestK(queryVertices_[threadID], findNearestKNeighbors + numSameVerticiesFound, graphNeighborhood);
  stateProperty_[queryVertices_[threadID]] = nullptr;

  // Loop through each neighbor until one is found that connects to the previous vertex
  bool foundValidConnToPrevious = false;
  // track if we added a vertex to the roadmapPath, so that we can remove it later if needed
  bool addedToRoadmapPath = false;
  TaskVertex candidateVertex;
  for (std::size_t neighborID = 0; neighborID < graphNeighborhood.size(); ++neighborID)
  {
    bool isValid = false;
    bool isRepeatOfPrevWaypoint = false;  // don't add current waypoint if same as last one

    // Find next state's vertex
    candidateVertex = graphNeighborhood[neighborID];

    // Check if next vertex is same as previous
    if (prevGraphVertex == candidateVertex)
    {
      // Do not do anything, we are done here
      foundValidConnToPrevious = true;
      if (verbose)
        std::cout << std::string(currVertexIndex, ' ') << "Previous vertex is same as current vertex, skipping "
                                                          "current vertex" << std::endl;

      isValid = true;
      isRepeatOfPrevWaypoint = true;
    }
    else
    {
      // Check for collision
      isValid = si_->checkMotion(stateProperty_[prevGraphVertex], stateProperty_[candidateVertex]);

      if (visualizeSnapPath_)  // Visualize
      {
        // Show the node we're currently considering going through
        visual_->viz5()->state(stateProperty_[candidateVertex], tools::MEDIUM, tools::PURPLE, 1);
        // edge between the state on the original inputPath and its neighbor we are currently considering
        double color = 25;  // light green
        visual_->viz5()->edge(currentPathState, stateProperty_[candidateVertex], color);

        color = isValid ? 75 : 100;  // orange, red
        // edge between the previous connection point we chose for the roadmapPath, and the currently considered
        // next state
        visual_->viz5()->edge(stateProperty_[prevGraphVertex], stateProperty_[candidateVertex], color);

        visual_->viz5()->trigger();
        usleep(visualizeSnapPathSpeed_ * 1000000);
      }

      if (isValid && verbose)  // Debug
        std::cout << std::string(currVertexIndex, ' ') << "Found valid nearby edge on loop " << neighborID << std::endl;
    }

    // Remember if any connections failed
    if (isValid)
    {
      if (neighborID > 0)
      {
        if (verbose)
          std::cout << std::string(currVertexIndex + 2, ' ') << "Found case where double loop fixed the "
                                                                "problem - loop " << neighborID << std::endl;
        // visual_->viz5()->trigger();
        // usleep(6*1000000);
      }
      foundValidConnToPrevious = true;

      // Add this waypoint solution
      if (!isRepeatOfPrevWaypoint)
      {
        // std::cout << std::string(currVertexIndex+2, ' ') << "roadmapPath.size=" << std::fixed <<
        // roadmapPath.size() << std::flush;
        // std::cout << " Vertex: " << candidateVertex;
        // std::cout << " State: " << stateProperty_[candidateVertex];
        // std::cout << std::endl;
        roadmapPath.push_back(candidateVertex);
        addedToRoadmapPath = true;  // remember it was added

        if (visualizeSnapPath_)  // Visualize
        {
          double color = 25;  // light green
          visual_->viz4()->edge(stateProperty_[prevGraphVertex], stateProperty_[candidateVertex], color);
          visual_->viz4()->trigger();
          usleep(visualizeSnapPathSpeed_ * 1000000);
        }
      }

      // Check if there are more points to process
      if (currVertexIndex + 1 >= inputPath.getStateCount())
      {
        if (verbose)
          std::cout << std::string(currVertexIndex, ' ') << "END OF PATH, great job :)" << std::endl;
        allValid = true;
        return true;
      }
      else
      {
        // Recurisvely call next level
        if (recurseSnapWaypoints(inputPath, roadmapPath, currVertexIndex + 1, candidateVertex, allValid, verbose))
        {
          return true;
        }
        else
        {
          // Keep trying to find a working neighbor, remove last roadmapPath node if we added one
          if (addedToRoadmapPath)
          {
            assert(roadmapPath.size() > 0);
            roadmapPath.pop_back();
            addedToRoadmapPath = false;
          }
        }
      }
    }
    else
    {
      if (verbose)
        std::cout << std::string(currVertexIndex, ' ') << "Loop " << neighborID << " not valid" << std::endl;
    }
  }  // for every neighbor

  if (!foundValidConnToPrevious)
  {
    std::cout << std::string(currVertexIndex, ' ') << "Unable to find valid connection to previous, backing up a "
                                                      "level and trying again" << std::endl;
    allValid = false;

    if (visualizeSnapPath_)  // Visualize
    {
      visual_->viz5()->trigger();
      usleep(visualizeSnapPathSpeed_ * 1000000);
    }

    // TODO(davetcoleman): remove hack
    std::cout << std::string(currVertexIndex, ' ') << "TODO remove this viz" << std::endl;

    // Show the node we're currently considering going through
    visual_->viz5()->state(stateProperty_[prevGraphVertex], tools::VARIABLE_SIZE, tools::PURPLE, 3);
    visual_->viz5()->trigger();
    usleep(0.001 * 1000000);

    return false;
  }
  std::cout << std::string(currVertexIndex, ' ') << "This loop found a valid connection, but higher recursive loop "
                                                    "(one that has already returned) did not" << std::endl;

  return false;  // this loop found a valid connection, but lower recursive loop did not
}

void TaskGraph::normalizeGraphEdgeWeights()
{
  bool verbose = false;

  if (!popularityBiasEnabled_)
  {
    OMPL_INFORM("Skipping normalize graph edge weights because not using popularity bias currently");
    return;
  }

  // Normalize weight of graph
  double total_cost = 0;
  foreach (TaskEdge e, boost::edges(g_))
  {
    total_cost += edgeWeightProperty_[e];
  }
  double avg_cost = total_cost / getNumEdges();

  if (verbose)
    OMPL_INFORM("Average cost of the edges in graph is %f", avg_cost);

  if (avg_cost < desiredAverageCost_)  // need to decrease cost in graph
  {
    double avgCostDiff = desiredAverageCost_ - avg_cost;

    if (verbose)
      std::cout << "avgCostDiff: " << avgCostDiff << std::endl;
    double perEdgeReduction = avgCostDiff;  // / getNumEdges();

    if (verbose)
      OMPL_INFORM("Decreasing each edge's cost by %f", perEdgeReduction);
    foreach (TaskEdge e, boost::edges(g_))
    {
      assert(edgeWeightProperty_[e] <= MAX_POPULARITY_WEIGHT);
      edgeWeightProperty_[e] = std::min(edgeWeightProperty_[e] + perEdgeReduction, MAX_POPULARITY_WEIGHT);
      if (verbose)
        std::cout << "Edge " << e << " now has weight " << edgeWeightProperty_[e] << " via reduction "
                  << perEdgeReduction << std::endl;
    }
  }
  else if (verbose)
  {
    OMPL_INFORM("Not decreasing all edge's cost because average is above desired");
  }
}

void TaskGraph::removeVertex(TaskVertex v)
{
  const bool verbose = false;

  if (verbose)
    std::cout << "Removing vertex " << v << std::endl;

  // Remove from nearest neighbor
  nn_->remove(v);

  // Delete state
  si_->freeState(stateProperty_[v]);
  stateProperty_[v] = nullptr;

  // Remove all edges to and from vertex
  boost::clear_vertex(v, g_);

  // Remove vertex
  boost::remove_vertex(v, g_);
}

void TaskGraph::checkStateType()
{
  std::size_t count = 0;
  foreach (const TaskVertex v, boost::vertices(g_))
  {
    // The first vertex (id=0) should have a nullptr state because it is used for searching
    if (!stateProperty_[v])
    {
      if (count != 0)
      {
        OMPL_ERROR("Null state found for vertex that is not zero");
      }
      continue;
    }

    // std::size_t level = getTaskLevel(stateProperty_[v]);
    // if (level > 2)
    // {
    //   OMPL_ERROR("State is on wrong level: %u", level);
    //   exit(-1);
    // }
  }
  OMPL_INFORM("All states checked for task level");
}

void TaskGraph::connectNewVertex(TaskVertex v1)
{
  bool verbose = false;

  // Visualize new vertex
  if (visualizeAddSample_)
  {
    visual_->viz1()->state(stateProperty_[v1], tools::SMALL, tools::GREEN, 1);
  }

  // Connect to nearby vertices
  std::vector<TaskVertex> graphNeighborhood;
  std::size_t findNearestKNeighbors = Discretizer::getEdgesPerVertex(si_);
  OMPL_INFORM("TaskGraph.connectNewVertex(): Finding %u nearest neighbors for new vertex", findNearestKNeighbors);
  const std::size_t numSameVerticiesFound = 1;  // add 1 to the end because the NN tree always returns itself

  // Search
  const std::size_t threadID = 0;
  stateProperty_[queryVertices_[threadID]] = stateProperty_[v1];
  nn_->nearestK(queryVertices_[threadID], findNearestKNeighbors + numSameVerticiesFound, graphNeighborhood);
  stateProperty_[queryVertices_[threadID]] =
      nullptr;  // Set search vertex to nullptr to prevent segfault on class unload of memory

  // For each nearby vertex, add an edge
  std::size_t errorCheckNumSameVerticies = 0;  // sanity check
  for (std::size_t i = 0; i < graphNeighborhood.size(); ++i)
  {
    TaskVertex &v2 = graphNeighborhood[i];

    // Check if these vertices are the same
    if (v1 == v2)
    {
      if (verbose)
        std::cout << "connectNewVertex attempted to connect edge to itself: " << v1 << ", " << v2 << std::endl;
      errorCheckNumSameVerticies++;  // sanity check
      continue;
    }

    // Check if these vertices are the same STATE
    if (si_->getStateSpace()->equalStates(stateProperty_[v1], stateProperty_[v2]))
    {
      OMPL_ERROR("This state has already been added, should not happen");
      exit(-1);
    }

    // Check if these vertices already share an edge
    if (boost::edge(v1, v2, g_).second)
    {
      std::cout << "skipped bc already share an edge " << std::endl;
      continue;
    }

    // Create edge if not in collision
    if (si_->checkMotion(stateProperty_[v1], stateProperty_[v2]))
    {
      // TaskEdge e =
      addEdge(v1, v2, desiredAverageCost_);
      // std::cout << "added valid edge " << e << std::endl;

      if (visualizeAddSample_)  // Debug: display edge
      {
        double popularity = 100;  // TODO: maybe make edge really popular so we can be sure its added to the
                                  // spars graph since we need it
        visual_->viz1()->edge(stateProperty_[v1], stateProperty_[v2], popularity);
      }
    }

  }  // for each neighbor

  // Make sure one and only one vertex is returned from the NN search that is the same as parent vertex
  BOOST_ASSERT_MSG(errorCheckNumSameVerticies <= numSameVerticiesFound, "Too many same verticies found ");

  // Visualize
  if (visualizeAddSample_)
  {
    visual_->viz1()->trigger();
    usleep(0.001 * 1000000);
  }

  // Record this new addition
  graphUnsaved_ = true;
}

void TaskGraph::removeInvalidVertices()
{
  OMPL_INFORM("Removing invalid vertices");
  bool actuallyRemove = true;
  std::size_t totalInvalid = 0;  // invalid states

  if (actuallyRemove)
    OMPL_WARN("Actually deleting verticies and resetting edge cache");

  typedef boost::graph_traits<TaskAdjList>::vertex_iterator VertexIterator;
  for (VertexIterator vertexIt = boost::vertices(g_).first; vertexIt != boost::vertices(g_).second; ++vertexIt)
  {
    if (*vertexIt <= queryVertices_.back())
      continue;

    if (*vertexIt % 1000 == 0)
      std::cout << "Checking vertex " << *vertexIt << std::endl;

    // Check if state is valid
    if (!si_->isValid(stateProperty_[*vertexIt]))
    {
      OMPL_ERROR("State %u is not valid", *vertexIt);
      totalInvalid++;

      visual_->viz5()->state(stateProperty_[*vertexIt], tools::ROBOT, tools::RED, 0);
      visual_->viz5()->trigger();
      // usleep(0.25 * 1000000);

      if (actuallyRemove)
      {
        removeVertex(*vertexIt);
        vertexIt--;  // backtrack one vertex
      }
    }
  }

  if (totalInvalid > 0)
  {
    OMPL_ERROR("Total invalid: %u", totalInvalid);

    if (actuallyRemove)
    {
      // Must clear out edge cache since we changed the numbering of vertices
      denseCache_->clear();
      graphUnsaved_ = true;
    }
  }
}
