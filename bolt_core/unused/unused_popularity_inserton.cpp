/** \brief Helper function for choosing the correct method for vertex insertion ordering */
void getVertexInsertionOrdering(std::list<WeightedVertex> &vertexInsertionOrder);

bool getPopularityOrder(std::list<WeightedVertex> &vertexInsertionOrder);
bool getDefaultOrder(std::list<WeightedVertex> &vertexInsertionOrder);
bool getRandomOrder(std::list<WeightedVertex> &vertexInsertionOrder);

void SparseCriteria::getVertexInsertionOrdering(std::list<WeightedVertex> &vertexInsertionOrder)
{
  if (sparseCreationInsertionOrder_ == 0)
  {
    OMPL_INFORM("Creating sparse graph using popularity ordering");
    getPopularityOrder(vertexInsertionOrder);  // Create SPARs graph in order of popularity
  }
  else if (sparseCreationInsertionOrder_ == 1)
  {
    OMPL_WARN("Creating sparse graph using default ordering");
    getDefaultOrder(vertexInsertionOrder);
  }
  else if (sparseCreationInsertionOrder_ == 2)
  {
    OMPL_WARN("Creating sparse graph using random ordering");
    getRandomOrder(vertexInsertionOrder);
  }
  else
  {
    OMPL_ERROR("Unknown insertion order method");
    exit(-1);
  }
}

bool SparseCriteria::getPopularityOrder(std::list<WeightedVertex> &vertexInsertionOrder)
{
  bool verbose = false;

  // Error check
  BOOST_ASSERT_MSG(getNumVertices() > queryVertices_.size(),
                   "Unable to get vertices in order of popularity because dense "
                   "graph is empty");

  if (visualizeNodePopularity_)  // Clear visualization
  {
    visual_->viz3()->deleteAllMarkers();
  }

  // Sort the vertices by popularity in a queue
  std::priority_queue<WeightedVertex, std::vector<WeightedVertex>, CompareWeightedVertex> pqueue;

  // Loop through each popular edge in the dense graph
  foreach (TaskVertex v, boost::vertices(g_))
  {
    // Do not process the search vertex, it is null
    if (v <= queryVertices_.back())
      continue;

    if (verbose)
      std::cout << "Vertex: " << v << std::endl;
    double popularity = 0;
    foreach (TaskVertex edge, boost::out_edges(v, g_))
    {
      if (verbose)
        std::cout << "  Edge: " << edge << std::endl;
      popularity += (100 - edgeWeightProperty_[edge]);
    }
    if (verbose)
      std::cout << "  Total popularity: " << popularity << std::endl;
    pqueue.push(WeightedVertex(v, popularity));
  }

  // Remember which one was the largest
  double largestWeight = pqueue.top().weight_;
  if (largestWeight == 0)
    largestWeight = 1;  // prevent division by zero

  if (verbose)
    std::cout << "Largest weight: " << largestWeight << std::endl
              << std::endl;

  // Convert pqueue into vector
  while (!pqueue.empty())  // Output the vertices in order
  {
    vertexInsertionOrder.push_back(pqueue.top());

    // Modify the weight to be a percentage of the max weight
    const double weightPercent = pqueue.top().weight_ / largestWeight * 100.0;
    vertexInsertionOrder.back().weight_ = weightPercent;

    // Visualize
    if (visualizeNodePopularity_)
    {
      if (verbose)
        std::cout << "vertex " << pqueue.top().v_ << ", mode 7, weightPercent " << weightPercent << std::endl;
      visual_->viz3()->state(stateProperty_[pqueue.top().v_], tools::SCALE, tools::BLACK, weightPercent);
    }

    // Remove from priority queue
    pqueue.pop();
  }

  if (visualizeNodePopularity_)
  {
    visual_->viz3()->trigger();
    usleep(0.001 * 1000000);
  }

  return true;
}

bool SparseCriteria::getDefaultOrder(std::list<WeightedVertex> &vertexInsertionOrder)
{
  bool verbose = false;
  double largestWeight = -1 * std::numeric_limits<double>::infinity();

  // Loop through each popular edge in the dense graph
  foreach (TaskVertex v, boost::vertices(g_))
  {
    // Do not process the search vertex, it is null
    if (v <= queryVertices_.back())
      continue;

    if (verbose)
      std::cout << "Vertex: " << v << std::endl;
    double popularity = 0;

    foreach (TaskVertex edge, boost::out_edges(v, g_))
    {
      if (verbose)
        std::cout << "  Edge: " << edge << std::endl;
      popularity += (100 - edgeWeightProperty_[edge]);
    }
    if (verbose)
      std::cout << "  Total popularity: " << popularity << std::endl;

    // Record
    vertexInsertionOrder.push_back(WeightedVertex(v, popularity));

    // Track largest weight
    if (popularity > largestWeight)
      largestWeight = popularity;
  }

  // Update the weights
  for (WeightedVertex wv : vertexInsertionOrder)
  {
    // Modify the weight to be a percentage of the max weight
    const double weightPercent = wv.weight_ / largestWeight * 100.0;
    wv.weight_ = weightPercent;

    // Visualize vertices
    if (visualizeNodePopularity_)
    {
      visual_->viz3()->state(stateProperty_[wv.v_], tools::SCALE, tools::BLACK, weightPercent);
    }
  }

  // Visualize vertices
  if (visualizeNodePopularity_)
  {
    visual_->viz3()->trigger();
    usleep(0.001 * 1000000);
  }

  return true;
}

bool SparseCriteria::getRandomOrder(std::list<WeightedVertex> &vertexInsertionOrder)
{
  std::list<WeightedVertex> defaultVertexInsertionOrder;
  getDefaultOrder(defaultVertexInsertionOrder);

  // Create vector and copy list into it
  std::vector<WeightedVertex> tempVector(defaultVertexInsertionOrder.size());
  std::copy(defaultVertexInsertionOrder.begin(), defaultVertexInsertionOrder.end(), tempVector.begin());

  // Randomize
  std::random_shuffle(tempVector.begin(), tempVector.end());

  // Convert back to list
  vertexInsertionOrder.resize(tempVector.size());
  std::copy(tempVector.begin(), tempVector.end(), vertexInsertionOrder.begin());

  return true;
}
