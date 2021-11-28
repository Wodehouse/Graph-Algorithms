/*
  Copyright (c) 2020
  Swarthmore College Computer Science Department, Swarthmore PA
  J. Brody, A. Danner, M. Gagne, L. Meeden, Z. Palmer, A. Soni, M. Wehar
  Distributed as course material for Fall 2020
  CPSC 035: Data Structures and Algorithms
  https://tinyurl.com/yyr8mdoh
*/

#pragma once

#include "stringUtils.h"

template <typename V, typename W>
AdjacencyListGraph<V, W>::AdjacencyListGraph() {
}

template <typename V, typename W>
AdjacencyListGraph<V, W>::~AdjacencyListGraph() {
    vector<V> sources = this->edges.getKeys();
    for (int i = 0; i < sources.size(); i++) {
        delete this->edges.get(sources[i]);
    }
}

template <typename V, typename W>
vector<V> AdjacencyListGraph<V, W>::getVertices() {
    return this->vertices.getKeys();
}

template <typename V, typename W>
void AdjacencyListGraph<V, W>::insertVertex(V v) {
    if (!this->vertices.contains(v)) {
        this->vertices.insert(v, true);
        this->edges.insert(v, new STLHashTable<V, W>());
    }
}

template <typename V, typename W>
void AdjacencyListGraph<V, W>::removeVertex(V v) {
    if (this->vertices.contains(v)) {
        this->vertices.remove(v);
    }
}

template <typename V, typename W>
bool AdjacencyListGraph<V, W>::containsVertex(V v) {
    return this->vertices.contains(v);
}

template <typename V, typename W>
void AdjacencyListGraph<V, W>::insertEdge(V src, V dest, W weight) {
    if (!containsVertex(src)) {
        throw runtime_error(std::string("Vertex ") + cs35::to_string(src) +
                            " not present in graph");
    }
    if (!containsVertex(dest)) {
        throw runtime_error(std::string("Vertex ") + cs35::to_string(dest) +
                            " not present in graph");
    }
    if (this->edges.get(src)->contains(dest)) {
        throw runtime_error(std::string("Edge already exists from ") +
                            cs35::to_string(src) + " to " +
                            cs35::to_string(dest));
    } else {
        this->edges.get(src)->insert(dest, weight);
    }
}

template <typename V, typename W>
void AdjacencyListGraph<V, W>::removeEdge(V src, V dest) {
    if (this->edges.contains(src) && this->edges.get(src)->contains(dest)) {
        this->edges.get(src)->remove(dest);
    } else {
        throw runtime_error(std::string("Edge does not exist from ") +
                            cs35::to_string(src) + " to " +
                            cs35::to_string(dest));
    }
}

template <typename V, typename W>
bool AdjacencyListGraph<V, W>::containsEdge(V source, V destination) {
    return this->edges.contains(source) &&
           this->edges.get(source)->contains(destination);
}

template <typename V, typename W>
Edge<V, W> AdjacencyListGraph<V, W>::getEdge(V source, V destination) {
    if (this->edges.contains(source) &&
        this->edges.get(source)->contains(destination)) {
        W weight = this->edges.get(source)->get(destination);
        return Edge<V, W>(source, destination, weight);
    } else {
        throw runtime_error(std::string("Edge does not exist from ") +
                            cs35::to_string(source) + " to " +
                            cs35::to_string(destination));
    }
}

template <typename V, typename W>
vector<Edge<V, W>> AdjacencyListGraph<V, W>::getEdges() {
    vector<Edge<V, W>> results;
    vector<V> sources = this->edges.getKeys();
    for (int i = 0; i < sources.size(); i++) {
        vector<V> targets = this->edges.get(sources[i])->getKeys();
        for (int j = 0; j < targets.size(); j++) {
            W weight = this->edges.get(sources[i])->get(targets[j]);
            results.push_back(Edge<V, W>(sources[i], targets[j], weight));
        }
    }
    return results;
}

template <typename V, typename W>
vector<Edge<V, W>> AdjacencyListGraph<V, W>::getOutgoingEdges(V source) {
    vector<Edge<V, W>> results;
    vector<V> targets = this->edges.get(source)->getKeys();
    for (int j = 0; j < targets.size(); j++) {
        W weight = this->edges.get(source)->get(targets[j]);
        results.push_back(Edge<V, W>(source, targets[j], weight));
    }
    return results;
}

template <typename V, typename W>
vector<Edge<V, W>>
AdjacencyListGraph<V, W>::getIncomingEdges(V destination) {
    vector<Edge<V, W>> results;
    vector<V> sources = this->edges.getKeys();
    for (int i = 0; i < sources.size(); i++) {
        if (this->edges.get(sources[i])->contains(destination)) {
            W weight = this->edges.get(sources[i])->get(destination);
            results.push_back(Edge<V, W>(sources[i], destination, weight));
        }
    }
    return results;
}

template <typename V, typename W>
vector<V> AdjacencyListGraph<V, W>::getNeighbors(V source) {
    if (this->edges.contains(source)) {
        return this->edges.get(source)->getKeys();
    } else {
        return vector<V>();
    }
}
