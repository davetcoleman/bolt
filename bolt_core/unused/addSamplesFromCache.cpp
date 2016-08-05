  void addSamplesFromCache(std::size_t indent);


void SparseCriteria::addSamplesFromCache(std::size_t indent)
{
  BOLT_CYAN_DEBUG(indent, vCriteria_, "addRandomSamplesFromCache()");
  indent += 2;

  if (denseCache_->getStateCacheSize() <= 1)
  {
    BOLT_WARN(indent, true, "Cache is empty, no states to add");
    return;
  }

  if (useDiscretizedSamples_)
  {
    BOLT_WARN(indent, true, "Not using cache for random samples because discretized samples is enabled.");
    return;
  }

  // Add from file if available - remember where it was
  std::size_t lastCachedStateIndex = denseCache_->getStateCacheSize() - 1;
  StateID candidateStateID = 1;  // skip 0, because that is the "deleted" NULL state ID

  while (candidateStateID <= lastCachedStateIndex)
  {
    if (!addSample(candidateStateID, indent))
      break;

    candidateStateID++;
  }  // end while
}
