void createSPARSOuterLoop();

  /** \brief Slight modification */
  bool spannerTestOuter(SparseVertex v, SparseVertex vp, SparseVertex vpp, InterfaceData& iData, std::size_t indent);

  /** \brief Using Astar to find shortest path */
  bool spannerTestAStar(SparseVertex v, SparseVertex vp, SparseVertex vpp, InterfaceData& iData, std::size_t indent);


void SparseCriteria::createSPARSOuterLoop()
{
  std::size_t indent = 2;

  // Reset parameters
  setup();
  visualizeOverlayNodes_ = false;  // DO NOT visualize all added nodes in a separate window
  denseCache_->resetCounters();

  // Get the ordering to insert vertices
  std::list<WeightedVertex> vertexInsertionOrder;
  getVertexInsertionOrdering(vertexInsertionOrder);

  // Error check order creation
  assert(vertexInsertionOrder.size() == getNumVertices() - queryVertices_.size());

  // Attempt to insert the vertices multiple times until no more succesful insertions occur
  secondSparseInsertionAttempt_ = false;
  std::size_t loopAttempt = 0;
  std::size_t sucessfulInsertions = 1;  // start with one so that while loop works
  while (sucessfulInsertions > 0)
  {
    std::cout << "Attempting to insert " << vertexInsertionOrder.size() << " vertices for the " << loopAttempt
              << " loop" << std::endl;

    // Sanity check
    if (loopAttempt > 3)
      OMPL_WARN("Suprising number of loop when attempting to insert nodes into SPARS graph: %u", loopAttempt);

    // Benchmark runtime
    time::point startTime = time::now();

    // ----------------------------------------------------------------------
    // Attempt to insert each vertex using the first 3 criteria
    if (!createSPARSInnerLoop(vertexInsertionOrder, sucessfulInsertions))
      break;

    // Benchmark runtime
    double duration = time::seconds(time::now() - startTime);

    // Visualize
    if (visualizeSparseGraph_)
      visual_->viz1()->trigger();

    std::cout << "Succeeded in inserting " << sucessfulInsertions << " vertices on the " << loopAttempt
              << " loop, remaining unin<serted vertices: " << vertexInsertionOrder.size()
              << " loop runtime: " << duration << " sec" << std::endl;
    loopAttempt++;

    // Increase the sparse delta a bit, but only after the first loop
    if (loopAttempt == 1)
    {
      // sparseDelta_ = getSecondarySparseDelta();
      std::cout << std::string(indent + 2, ' ') << "sparseDelta_ is now " << sparseDelta_ << std::endl;
      secondSparseInsertionAttempt_ = true;

      // Save collision cache, just in case there is a bug
      denseCache_->save();
    }

    bool debugOverRideJustTwice = true;
    if (debugOverRideJustTwice && loopAttempt == 1)
    {
      OMPL_WARN("Only attempting to add nodes twice for speed");
      break;
    }
  }

  // If we haven't animated the creation, just show it all at once
  if (!visualizeSparseGraph_)
  {
    sg_->displayDatabase(true, indent+4);
  }
  else if (sg_->visualizeSparseGraphSpeed_ < std::numeric_limits<double>::epsilon())
  {
    visual_->viz1()->trigger();
    usleep(0.001 * 1000000);
  }
}

  bool SparseCriteria::spannerTestOuter(SparseVertex v, SparseVertex vp, SparseVertex vpp, InterfaceData & iData,
                                        std::size_t indent)
  {
    BOLT_FUNC(indent, vCriteria_, "spannerTestOuter()");
    // Computes all nodes which qualify as a candidate x for v, v', and v"
    double midpointPathLength = maxSpannerPath(v, vp, vpp, indent + 2);

    // Must have both interfaces to continue
    if (!iData.hasInterface1() || !iData.hasInterface2())
    {
      return false;
    }

    double newDistance = si_->distance(iData.getInterface1Outside(),
                                       iData.getInterface2Outside());  // TODO(davetcoleman): cache?
    BOOST_ASSERT_MSG(newDistance > 0, "Distance between outer interface vertices cannot be 0");

    if (stretchFactor_ * newDistance < midpointPathLength)
    {
      BOLT_WARN(indent + 2, vQuality_, "Spanner property violated");
      BOLT_DEBUG(indent + 4, vQuality_, "Sparse Graph Midpoint Length  = " << midpointPathLength);
      BOLT_DEBUG(indent + 4, vQuality_, "Spanner Path Length * Stretch = " << (stretchFactor_ * newDistance));
      BOLT_DEBUG(indent + 6, vQuality_, "new distance = " << newDistance);
      BOLT_DEBUG(indent + 6, vQuality_, "stretch factor = " << stretchFactor_);

      return true;  // spannerPropertyWasViolated
    }
    else
      BOLT_DEBUG(indent + 4, vQuality_, "Spanner property not violated");

    return false;  // spannerPropertyWasViolated = false
  }


  bool SparseCriteria::spannerTestAStar(SparseVertex v, SparseVertex vp, SparseVertex vpp, InterfaceData & iData,
                                        std::size_t indent)
  {
    BOLT_FUNC(indent, vQuality_, "spannerTestAStar()");

    if (!iData.hasInterface1() || !iData.hasInterface2())
    {
      BOLT_ERROR(indent, vQuality_ || true, "No interface between points");
      return false;
    }

    // Experimental calculations
    double pathLength = 0;
    std::vector<SparseVertex> vertexPath;
    if (!sg_->astarSearch(vp, vpp, vertexPath, pathLength, indent))
    {
      BOLT_ERROR(indent, vQuality_, "No path found");
      visual_->waitForUserFeedback("No path found");
    }
    else
    {
      if (visualizeQualityCriteriaAstar_)
      {
        visual_->viz6()->deleteAllMarkers();
        assert(vertexPath.size() > 1);
        for (std::size_t i = 1; i < vertexPath.size(); ++i)
        {
          visual_->viz6()->edge(sg_->getState(vertexPath[i - 1]), sg_->getState(vertexPath[i]), tools::MEDIUM,
                                tools::GREEN);
        }
      }

      // Add connecting segments:
      double connector1 = si_->distance(sg_->getState(vp), iData.getOutsideInterfaceOfV1(vp, vpp));
      // TODO(davetcoleman): may want to include the dist from inside to outside
      // of interface
      BOLT_DEBUG(indent, vQuality_, "connector1 " << connector1);
      if (visualizeQualityCriteriaAstar_)
      {
        visual_->viz6()->edge(sg_->getState(vp), iData.getOutsideInterfaceOfV1(vp, vpp), tools::MEDIUM, tools::ORANGE);
      }

      double connector2 = si_->distance(sg_->getState(vpp), iData.getOutsideInterfaceOfV2(vp, vpp));
      // TODO(davetcoleman): may want to include the dist from inside to outside
      // of interface
      BOLT_DEBUG(indent, vQuality_, "connector2 " << connector2);
      if (visualizeQualityCriteriaAstar_)
      {
        visual_->viz6()->edge(sg_->getState(vpp), iData.getOutsideInterfaceOfV2(vp, vpp), tools::MEDIUM, tools::YELLOW);
      }

      pathLength += connector1 + connector2;
      BOLT_DEBUG(indent, vQuality_, "Full Path Length: " << pathLength);

      visual_->viz6()->trigger();
    }

    // Theoretical max
    // double theoreticalMaxLength = iData.getLastDistance() + 2 * sparseDelta_;
    // TODO is this right?
    double theoreticalMaxLength = iData.getLastDistance() + sparseDelta_;
    BOLT_DEBUG(indent, vQuality_, "Max allowable length: " << theoreticalMaxLength);
    theoreticalMaxLength *= stretchFactor_;
    BOLT_DEBUG(indent, vQuality_, "Max allowable length with stretch: " << theoreticalMaxLength);

    if (pathLength >= theoreticalMaxLength)
    {
      if (visualizeQualityCriteria_)                           // TEMP
        visualizeCheckAddPath(v, vp, vpp, iData, indent + 4);  // TEMP

      visual_->waitForUserFeedback("astar rule");

      return true;  // spannerPropertyWasViolated = true
    }

    BOLT_DEBUG(indent, vQuality_, "Astar says we do not need to add an edge");
    visual_->waitForUserFeedback("astar rule");

    return false;  // spannerPropertyWasViolated = false
  }
