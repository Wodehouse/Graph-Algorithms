/*
  Copyright (c) 2020
  Swarthmore College Computer Science Department, Swarthmore PA
  J. Brody, A. Danner, M. Gagne, L. Meeden, Z. Palmer, A. Soni, M. Wehar
  Distributed as course material for Fall 2020
  CPSC 035: Data Structures and Algorithms
  https://tinyurl.com/yyr8mdoh
*/

#include <stdexcept>
#include "adts/stlHashTable.h"
#include "adts/stlStack.h"
#include "adjacencyListGraph.h"
#include "adts/stlQueue.h"
#include "adts/stlMinPriorityQueue.h"
#include "adts/edge.h"

template <typename V, typename W>
bool reachableDFS(Graph<V, W>* g, V src, V dest) {
    //create a new stack...STLStack<T>
    Stack<V>* oink = new STLStack<V>();
    //create a set to store visited vertices-dictionary-hashtable: STLHashTable
    Dictionary<V, bool>* visited = new STLHashTable<V, bool>();
    oink->push(src);
    visited->insert(src, true);
    while(!oink->isEmpty()){
        V current = oink->pop();
        if(current == dest){
            delete visited;
            delete oink;
            return true;
        }
        vector<V> neighbors = g->getNeighbors(current);
        for(int i=0; i < neighbors.size(); i++){
            if(!visited->contains(neighbors[i])){
                oink->push(neighbors[i]);
                visited->insert(neighbors[i], true);
            }
        }
    }
    delete visited;
    delete oink;
    return false;
    
    //throw std::runtime_error("Not yet implemented: reachableDFS");
}

template <typename V, typename W>
vector<V> shortestLengthPathBFS(Graph<V, W>* g, V src, V dest) {
    Queue<V>* bacon = new STLQueue<V>();
    Dictionary<V, V>* previous = new STLHashTable<V, V>();
    bacon->enqueue(src);
    previous->insert(src, src);
    while(!bacon->isEmpty()){
        V current = bacon->dequeue();
        if(current == dest) {
            vector<V> soln;
            soln.push_back(current);
            while(current != src){
                //reconstruct path by following previous 'pointers'
                //current is in previous
                //previous->get(current) returns vertex pointing to current
                //return path
                current = previous->get(current);
                soln.push_back(current);
            }
            delete bacon;
            delete previous;
            return soln;
        }
        vector<V> neighbors = g->getNeighbors(current);
        for(int i=0; i < neighbors.size(); i++){
            if(!previous->contains(neighbors[i])){
                previous->insert(neighbors[i], current);
                bacon->enqueue(neighbors[i]);                
            }
        }
    }
    throw std::runtime_error("Path Does Not Exist!");
}

template <typename V, typename W>
Dictionary<V, W>* singleSourceShortestPath(Graph<V, W>* g, V src) {
    PriorityQueue<W, V>* instaham = new STLMinPriorityQueue<W, V>();
    Dictionary<V, W>* costs = new STLHashTable<V, W>();
    instaham->insert(0,src);
    costs->insert(src, 0);
    while(!instaham->isEmpty()){
        V current = instaham->remove();
        vector<Edge<V, W>> outEdges = g->getOutgoingEdges(src);
        //vector<V> neighbors = g->getNeighbors(current);
        
        for(int i = 0; i < outEdges.size(); i++){ //outEdges[i].dest
            V neighbor = outEdges[i].getDestination();
            int weight = outEdges[i].getWeight();
            int newCost = costs->get(current) + weight; 
             if(!costs->contains(neighbor)){
                instaham->insert(newCost, neighbor); 
                costs->insert(neighbor, newCost); 
             }
             else if(costs->get(neighbor) > newCost){
                instaham->insert(newCost, neighbor); 
                costs->update(neighbor, newCost); 
             }
        }  
    }
    delete instaham; 
    return costs;
    //throw std::runtime_error("Not yet implemented: singleSourceShortestPath");
}
