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
    if (argc != 4) {
        cout << "This program expects three arguments:" << endl;
        cout << "  * The name of the file containing a social network graph."
             << endl;
        cout << "  * The name of the first person to check." << endl;
        cout << "  * The name of the second person to check." << endl;
        cout << "Example: ./oinkedIn test_data/socialNetwork_8.graph "
             << "\"Kevin Bacon\" \"Jaimie Dengar\"" << endl;
        return 1;
    }

    string filename = argv[1];
    string pig1 = argv[2];
    string pig2 = argv[3];

    Graph<string,int>* socialNetwork;
    try {
        socialNetwork = loadGraph<string,int>(filename, false);
    } catch (runtime_error e) {
        cout << "Could not read graph file: " << e.what() << endl;
        return 2;
    }

    if (!socialNetwork->containsVertex(pig1)) {
        cout << "No one named \"" << pig1 << "\" appears in that network."
             << endl;
        return 3;
    }

    if (!socialNetwork->containsVertex(pig2)) {
        cout << "No one named \"" << pig2 << "\" appears in that network."
             << endl;
        return 4;
    }

    if (reachableDFS(socialNetwork, pig1, pig2)) {
        cout << pig1 << " and " << pig2 << " are connected." << endl;
    } else {
        cout << pig1 << " and " << pig2 << " are not connected." << endl;
    }

    delete socialNetwork;
    return 0;
}