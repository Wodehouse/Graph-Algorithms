#pragma once
/*
  Copyright (c) 2020
  Swarthmore College Computer Science Department, Swarthmore PA
  J. Brody, A. Danner, M. Gagne, L. Meeden, Z. Palmer, A. Soni, M. Wehar
  Distributed as course material for Spring 2020
  CPSC 035: Data Structures and Algorithms
  https://tinyurl.com/vdxo2w8
*/

/**
 * An Edge is a class that represents a directed, weighted edge in a graph.
 * @tparam V a type to represent the vertex labels
 * @tparam W a type to represent the weight (usually an int, float, or double)
 */
template <typename V, typename W> class Edge {
  public:
    Edge(V source, V destination, W weight = 1);

    V getSource() {
        return source;
    }
    V getDestination() {
        return dest;
    }
    W getWeight() {
        return weight;
    }

  private:
    V source;
    V dest;
    W weight;
};

template <typename V, typename W>
Edge<V, W>::Edge(V source, V dest, W weight) {
    this->source = source;
    this->weight = weight;
    this->dest = dest;
}
