/*
  Copyright (c) 2020
  Swarthmore College Computer Science Department, Swarthmore PA
  J. Brody, A. Danner, M. Gagne, L. Meeden, Z. Palmer, A. Soni, M. Wehar
  Distributed as course material for Fall 2020
  CPSC 035: Data Structures and Algorithms
  https://tinyurl.com/yyr8mdoh
*/

#pragma once

#include "adts/edge.h"
#include "adts/graph.h"
#include "adts/stlHashTable.h"

template <typename V, typename W>
class AdjacencyListGraph : public Graph<V, W> {
  public:
    AdjacencyListGraph();
    virtual ~AdjacencyListGraph();

    virtual vector<V> getVertices();
    virtual void insertVertex(V v);
    virtual void removeVertex(V v);
    virtual bool containsVertex(V v);
    virtual void insertEdge(V src, V dest, W weight);
    virtual void removeEdge(V src, V dest);
    virtual bool containsEdge(V source, V destination);
    virtual Edge<V, W> getEdge(V source, V destination);
    virtual vector<Edge<V, W>> getEdges();
    virtual vector<Edge<V, W>> getOutgoingEdges(V source);
    virtual vector<Edge<V, W>> getIncomingEdges(V destination);
    virtual vector<V> getNeighbors(V source);

  private:
    STLHashTable<V, bool> vertices;
    STLHashTable<V, STLHashTable<V, W>*> edges;
};

#include "adjacencyListGraph-inl.h"
