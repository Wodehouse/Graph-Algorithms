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
        cout << "  * The name of the file containing a town map."
             << endl;
        cout << "  * The name of the location to analyze." << endl;
        cout << "Example: ./instaham test_data/town_8.graph "
             << "\"Pigma's Jewelry Museum\"" << endl;
        return 1;
    }

    string filename = argv[1];
    string location = argv[2];

    Graph<string,float>* town;
    try {
        town = loadGraph<string,float>(filename, false);
    } catch (runtime_error e) {
        cout << "Could not read graph file: " << e.what() << endl;
        return 2;
    }

    if (!town->containsVertex(location)) {
        cout << "No location named \"" << location << "\" appears in that town."
             << endl;
        return 3;
    }

    Dictionary<string,float>* results =
        singleSourceShortestPath(town, location);
    vector<pair<string,float>> items = results->getItems();
    cout << location << " can reach " << items.size() << " location"
         << (items.size()==1?"":"s") << "." << endl;
    for (int i=0;i<items.size();i++) {
        cout << location << " can reach " << items[i].first << " in "
                << items[i].second << "." << endl;
    }
    delete results;

    delete town;
    return 0;
}