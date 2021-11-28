#pragma once

/*
  Copyright (c) 2020
  Swarthmore College Computer Science Department, Swarthmore PA
  J. Brody, A. Danner, M. Gagne, L. Meeden, Z. Palmer, A. Soni, M. Wehar
  Distributed as course material for Fall 2020
  CPSC 035: Data Structures and Algorithms
  https://tinyurl.com/yyr8mdoh
*/

#include <fstream>
#include <string>
#include <unordered_map>
#include <utility>

#include "adts/dictionary.h"
#include "adts/graph.h"
#include "adts/list.h"

/**
 * Loads a graph from a file into an existing graph object.  (The caller should
 * probably provide an empty, newly-allocated graph object.)  Additional data
 * about the vertices will be loaded into the provided dictionary (which should
 * probably also be empty and newly-allocated).
 * @param filename The name of the file to load.
 * @param directed True if the graph is directed; false if it is undirected.
 * @throws runtime_error If an I/O error occurs.
 */
template <typename V, typename W>
Graph<V,W>* loadGraph(std::string filename, bool directed);

#include "graphIO-inl.h"
