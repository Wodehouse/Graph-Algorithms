/*
  Copyright (c) 2020
  Swarthmore College Computer Science Department, Swarthmore PA
  J. Brody, A. Danner, M. Gagne, L. Meeden, Z. Palmer, A. Soni, M. Wehar
  Distributed as course material for Fall 2020
  CPSC 035: Data Structures and Algorithms
  https://tinyurl.com/yyr8mdoh
*/

#include <iostream>

#include "adts/graph.h"
#include "graphAlgorithms.h"
#include "graphIO.h"

using namespace std;

int main(int argc, char** argv) {
    if (argc != 3) {
        cout << "This program expects two arguments:" << endl;
        cout << "  * The name of the file containing a social network graph."
             << endl;
        cout << "  * The name of the person to connect to Kevin Bacon." << endl;
        cout << "Example: ./bacon test_data/socialNetwork_8.graph "
             << "\"Jaimie Dengar\"" << endl;
        return 1;
    }

    string filename = argv[1];
    string destPig = argv[2];

    Graph<string,int>* socialNetwork;
    try {
        socialNetwork = loadGraph<string,int>(filename, false);
    } catch (runtime_error e) {
        cout << "Could not read graph file: " << e.what() << endl;
        return 2;
    }

    if (!socialNetwork->containsVertex(destPig)) {
        cout << "No one named \"" << destPig << "\" appears in that network."
             << endl;
        return 3;
    }

    if (!socialNetwork->containsVertex("Kevin Bacon")) {
        cout << "Kevin Bacon does not appear in that network." << endl;
        return 4;
    }

    try {
        string src = "Kevin Bacon";
        vector<string> path = shortestLengthPathBFS(
            socialNetwork, src, destPig);
        int distance = path.size()-1;
        for (int i=0;i<distance;i++) {
            cout << path[i] << " knows " << path[i+1] << "." << endl;
        }
        cout << destPig << " is " << distance << " step"
             << (distance==1?"":"s") << " from Kevin Bacon." << endl;
    } catch (runtime_error e) {
        cout << destPig << " is not connected to Kevin Bacon." << endl;
    }

    delete socialNetwork;
    return 0;
}