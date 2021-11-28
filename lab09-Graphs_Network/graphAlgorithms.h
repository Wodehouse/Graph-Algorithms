/*
  Copyright (c) 2020
  Swarthmore College Computer Science Department, Swarthmore PA
  J. Brody, A. Danner, M. Gagne, L. Meeden, Z. Palmer, A. Soni, M. Wehar
  Distributed as course material for Fall 2020
  CPSC 035: Data Structures and Algorithms
  https://tinyurl.com/yyr8mdoh
*/

#pragma once

#include <vector>
using std::vector;

#include "adts/dictionary.h"
#include "adts/edge.h"
#include "adts/graph.h"

/**
 * Returns whether vertex dest is reachable from vertex src in the graph g using
 * depth-first search.
 * @param g A pointer to the Graph object.
 * @param src The source vertex.
 * @param dest The destination vertex.
 * @return true if a path exists
 */
template <typename V, typename W>
bool reachableDFS(Graph<V, W>* g, V src, V dest);

/**
 * Returns the shortest length path found from the vertex src
 * to the vertex dest, using an unweighted breadth-first search of the graph.
 * @param g - pointer to Graph object
 * @param src - source vertex
 * @param dest - destination vertex
 * @return vector<V> - sequence of vertices in path from src to dest
 * @error throws runtime_error if no path exists
 */
template <typename V, typename W>
vector<V> shortestLengthPathBFS(Graph<V, W>* g, V src, V dest);

/**
 * Dijkstra's algorithm calculates the single source minimum cost to all
 * other vertices in the graph from a src node
 * @param g - pointer to Graph object to search
 * @param src - source vertex
 * @return Dictionary<V,W> - Dictionary of vertices and their distance
 *    from src.  Only reachable vertices should appear in this dictionary.
 */
template <typename V, typename W>
Dictionary<V, W>* singleSourceShortestPath(Graph<V, W>* g, V src);

#include "graphAlgorithms-inl.h"
