  /** \brief After adding a new vertex, check if there is a really close nearby vertex that can be merged with this one
   */
  bool checkRemoveCloseVertices(SparseVertex v1, std::size_t indent = 0);
  void visualizeRemoveCloseVertices(SparseVertex v1, SparseVertex v2);



bool SparseCriteria::checkRemoveCloseVertices(SparseVertex v1, std::size_t indent)
{
  // This feature can be optionally disabled
  if (!useCheckRemoveCloseVertices_)
    return false;

  BOLT_FUNC(indent, vRemoveClose_, "checkRemoveCloseVertices() v1 = " << v1);

  // Get neighbors
  std::vector<SparseVertex> graphNeighbors;
  const std::size_t numNeighbors = 2;  // the first neighbor is (usually?) the vertex itself
  sg_->getNN()->nearestK(v1, numNeighbors, graphNeighbors);

  if (vRemoveClose_)
  {
    std::stringstream ss;
    std::copy(graphNeighbors.begin(), graphNeighbors.end(), std::ostream_iterator<SparseVertex>(ss, ", "));
    BOLT_DEBUG(indent, vRemoveClose_, "Neighbors of " << v1 << " are: [" << ss.str() << "]");
  }

  // Error check no neighbors
  if (graphNeighbors.size() <= 1)
  {
    BOLT_ERROR(indent, vRemoveClose_, "checkRemoveCloseVertices: no neighbors found for sparse vertex " << v1);
    return false;
  }

  // TODO: the following line assumes the first neighbor is itself, but I don't think this is always true
  SparseVertex v2 = graphNeighbors[1];

  // Error check: Do not remove itself
  if (v1 == v2)
  {
    BOLT_ERROR(indent, vRemoveClose_, "checkRemoveCloseVertices: error occured, cannot remove self: " << v1);
    exit(-1);
  }

  // Check that nearest neighbor is not an quality node - these should not be moved
  if (sg_->getVertexTypeProperty(v2) == QUALITY)
  {
    if (visualizeRemoveCloseVertices_)
    {
      visualizeRemoveCloseVertices(v1, v2);
      visual_->waitForUserFeedback("Skipping this vertex because is QUALITY");
    }
    return false;
  }

  // Check if nearest neighbor is within distance threshold
  if (sg_->distanceFunction(v1, v2) > sparseDelta_ * sparseDeltaFractionCheck_)
  {
    // BOLT_DEBUG(indent, vRemoveClose_, "Distance " << sg_->distanceFunction(v1, v2) << " is greater than max "
    //<< sparseDelta_ * sparseDeltaFractionCheck_);
    return false;
  }

  // Check if nearest neighbor is collision free
  if (!si_->checkMotion(sg_->getState(v1), sg_->getState(v2)))
  {
    BOLT_ERROR(indent, vRemoveClose_, "checkRemoveCloseVertices: not collision free v1=" << v1 << ", v2=" << v2);
    return false;
  }

  BOLT_DEBUG(indent, vRemoveClose_, "Found visible nearby node, testing if able to replace " << v2 << " with " << v1);

  // Nearest neighbor is good candidate, next check if all of its connected
  // neighbors can be connected to new vertex
  foreach (SparseEdge edge, boost::out_edges(v2, sg_->getGraph()))
  {
    SparseVertex v3 = boost::target(edge, sg_->getGraph());
    BOLT_DEBUG(indent + 2, vRemoveClose_, "checking edge v1= " << v1 << " to v3=" << v3);

    // Check if distance is within sparseDelta
    if (si_->distance(sg_->getState(v1), sg_->getState(v3)) > sparseDelta_)
    {
      BOLT_DEBUG(indent + 2, vRemoveClose_, "checkRemoveCloseVertices: distance too far from previous neighbor " << v3);
      return false;
    }

    // Check if collision free path to connected vertex
    if (!si_->checkMotion(sg_->getState(v1), sg_->getState(v3)))
    {
      BOLT_ERROR(indent + 2, vRemoveClose_,
                 "checkRemoveCloseVertices: not collision free path from new vertex to potential neighbor " << v3);
      return false;
    }
  }

  BOLT_DEBUG(indent, vRemoveClose_, "Found qualified node to replace with nearby");

  if (visualizeRemoveCloseVertices_)
  {
    visualizeRemoveCloseVertices(v1, v2);
    visual_->waitForUserFeedback("found qualified node to replace with nearby");
  }

  // Remove all interface data for old state
  sg_->clearInterfaceData(sg_->getStateNonConst(v2));

  // Connect new vertex to old vertex's connected neighbors
  foreach (SparseEdge edge, boost::out_edges(v2, sg_->getGraph()))
  {
    SparseVertex v3 = boost::target(edge, sg_->getGraph());
    BOLT_DEBUG(indent + 2, vRemoveClose_, "Connecting v1= " << v1 << " to v3=" << v3);
    sg_->addEdge(v1, v3, eINTERFACE, indent + 4);
  }

  // Delete old vertex
  sg_->removeVertex(v2, indent);
  BOLT_DEBUG(indent, vRemoveClose_, "REMOVING VERTEX " << v2 << " which was replaced with " << v1 << " with state ");

  // Statistics
  numVerticesMoved_++;

  // Only display database if enabled
  if (sg_->visualizeSparseGraph_ && sg_->visualizeSparseGraphSpeed_ > std::numeric_limits<double>::epsilon())
    sg_->displayDatabase(true, true, 1, indent);

  // if (visualizeRemoveCloseVertices_)
  // visual_->waitForUserFeedback("finished moving vertex");

  if (visualizeRemoveCloseVertices_)
  {
    visual_->viz6()->deleteAllMarkers();
    visual_->viz6()->trigger();
    usleep(0.001 * 1000000);
  }

  return true;
}

void SparseCriteria::visualizeRemoveCloseVertices(SparseVertex v1, SparseVertex v2)
{
  visual_->viz6()->deleteAllMarkers();
  visual_->viz6()->state(sg_->getState(v1), tools::LARGE, tools::GREEN, 0);
  visual_->viz6()->state(sg_->getState(v2), tools::LARGE, tools::RED, 0);  // RED = to be removed
  visual_->viz6()->trigger();
  usleep(0.001 * 1000000);
}
