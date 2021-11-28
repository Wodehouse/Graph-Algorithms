/*
  Copyright (c) 2020
  Swarthmore College Computer Science Department, Swarthmore PA
  J. Brody, A. Danner, M. Gagne, L. Meeden, Z. Palmer, A. Soni, M. Wehar
  Distributed as course material for Fall 2020
  CPSC 035: Data Structures and Algorithms
  https://tinyurl.com/yyr8mdoh
*/

#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

#include <UnitTest++/UnitTest++.h>

#include "adjacencyListGraph.h"
#include "adts/dictionary.h"
#include "adts/graph.h"
#include "graphAlgorithms.h"
#include "graphIO.h"

using namespace std;

template <typename V, typename W>
Graph<V, W>* readGraphFile(string filename) {
    return loadGraph<V, W>(filename, false);
}

/* helper check function that verifies there are no
   duplicate values in a vector */
template <typename T> void noDupesCheck(std::vector<T> expr);

/* helper check function that verifies that
   two vectors are set equivalent. An item
   is in the first vector iff it is in the
   second vector.  */
template <typename T>
void vectorsEqualCheck(std::vector<T> first, std::vector<T> second);

/* helper function to check if two paths are equivalent */
void checkEqualPath(vector<string> expected, vector<string> actual);

/** helper check function that verifies, given a graph, a shortest path, and an
    example path, that the example path is a correct shortest path in the graph
*/
template <typename W>
void checkShortestPath(Graph<string, W>* graph, vector<string> expected,
                       vector<string> actual);

template <typename V, typename W>
Graph<V, W>* makeGraph(vector<V> vertices, vector<Edge<V, W>> edges,
                       bool directed = false) {
    Graph<V, W>* graph = new AdjacencyListGraph<V, W>();
    for (int i = 0; i < vertices.size(); i++) {
        graph->insertVertex(vertices.at(i));
    }
    for (int i = 0; i < edges.size(); i++) {
        Edge<V, W> edge = edges.at(i);
        graph->insertEdge(edge.getSource(), edge.getDestination(),
                          edge.getWeight());
        if (!directed) {
            graph->insertEdge(edge.getDestination(), edge.getSource(),
                              edge.getWeight());
        }
    }
    return graph;
}

TEST(dfsTwoVertexGraph) {
    // This creates the *undirected* graph:
    //    "1"  --- 1 ---  "2"
    Graph<string, int>* graph =
        makeGraph<string, int>({"1", "2"}, {{"1", "2", 1}});
    CHECK(reachableDFS(graph, string("1"), string("2")));
    CHECK(reachableDFS(graph, string("2"), string("1")));
    delete graph;
}

TEST(dfsTwoDisconnectedVertices) {
    // This creates the *undirected* graph:
    //    "1"                  "2"
    Graph<string, int>* graph =
        makeGraph<string, int>({"1", "2"}, {});
    CHECK(!reachableDFS(graph, string("1"), string("2")));
    CHECK(!reachableDFS(graph, string("2"), string("1")));
    delete graph;
}

TEST(dfsThreeVertexDirectedGraph) {
    // This call makes a *directed* graph!
    Graph<string, int>* graph =
        makeGraph<string, int>(
            {"1", "2", "3"}, {
                                 {"1", "2", 1},
                                 {"1", "3", 1},
                             }, true);
    CHECK(reachableDFS(graph, string("1"), string("1")));
    CHECK(reachableDFS(graph, string("1"), string("2")));
    CHECK(reachableDFS(graph, string("1"), string("3")));
    CHECK(!reachableDFS(graph, string("3"), string("1")));
    CHECK(!reachableDFS(graph, string("2"), string("1")));
    CHECK(!reachableDFS(graph, string("2"), string("3")));
    delete graph;
}

TEST(grading_dfs_socialNetwork_2) {
    Graph<string, int>* graph = readGraphFile<string,int>(
        "test_data/socialNetwork_2.graph");
    CHECK(reachableDFS(graph, string("Kevin Bacon"), string("Gryllus Floyd")));
    CHECK(reachableDFS(graph, string("Kevin Bacon"), string("Kevin Bacon")));
    CHECK(reachableDFS(graph, string("Gryllus Floyd"), string("Kevin Bacon")));
    CHECK(reachableDFS(graph, string("Gryllus Floyd"), string("Gryllus Floyd")));
    delete graph;
}

TEST(grading_dfs_socialNetwork_4) {
    Graph<string, int>* graph = readGraphFile<string,int>(
        "test_data/socialNetwork_4.graph");
    CHECK(reachableDFS(graph, string("Gryllus Floyd"), string("Gryllus Floyd")));
    CHECK(reachableDFS(graph, string("Jared Watson"), string("Jared Watson")));
    CHECK(!reachableDFS(graph, string("Ace Watson"), string("Gryllus Floyd")));
    CHECK(!reachableDFS(graph, string("Kevin Bacon"), string("Jared Watson")));
    delete graph;
}

TEST(grading_dfs_socialNetwork_8) {
    Graph<string, int>* graph = readGraphFile<string,int>(
        "test_data/socialNetwork_8.graph");
    CHECK(reachableDFS(graph, string("Wibbly Bajie"), string("Jaimie Dengar")));
    CHECK(reachableDFS(graph, string("Jaimie Dengar"), string("Gryllus Floyd")));
    CHECK(reachableDFS(graph, string("Ace Watson"), string("Jaimie Dengar")));
    delete graph;
}

TEST(grading_dfs_socialNetwork_25) {
    Graph<string, int>* graph = readGraphFile<string,int>(
        "test_data/socialNetwork_25.graph");
    CHECK(reachableDFS(graph, string("Ace Watson"), string("Jodie Robinson")));
    CHECK(reachableDFS(graph, string("Jodie Robinson"), string("Karnac Wen")));
    CHECK(reachableDFS(graph, string("Jaimie Dengar"), string("Peppa Hog")));
    CHECK(reachableDFS(graph, string("Edessa Porker"), string("Fener Trwyth")));
    CHECK(reachableDFS(graph, string("Grunty Dengar"), string("Toby Piggins")));
    CHECK(reachableDFS(graph, string("Waddles Hog"), string("Karnac Wen")));
    delete graph;
}

TEST(grading_dfs_socialNetwork_100) {
    Graph<string, int>* graph = readGraphFile<string,int>(
        "test_data/socialNetwork_100.graph");
    CHECK(reachableDFS(graph, string("Zhu Stele"), string("Erymanthian Rosso")));
    CHECK(reachableDFS(graph, string("Orson Dengar"), string("Brad Danseur")));
    CHECK(reachableDFS(graph, string("Freddy Pigasus"), string("Freddy Stele")));
    CHECK(reachableDFS(graph, string("Brad Danseur"), string("Peppa Hog")));
    CHECK(reachableDFS(graph, string("Hen Pettitoes"), string("Brad Trwyth")));
    CHECK(reachableDFS(graph, string("Edessa Porker"), string("Freddy Stele")));
    CHECK(reachableDFS(graph, string("Okkoto Bacon"), string("Hen Ham")));
    CHECK(reachableDFS(graph, string("Francis Wen"), string("Orson Robinson")));
    delete graph;
}

TEST(grading_dfs_socialNetworkDisconnected_4) {
    Graph<string, int>* graph = readGraphFile<string,int>(
        "test_data/socialNetworkDisconnected_4.graph");
    CHECK(reachableDFS(graph, string("Kevin Bacon"), string("Gryllus Floyd")));
    CHECK(!reachableDFS(graph, string("Jared Watson"), string("Kevin Bacon")));
    CHECK(reachableDFS(graph, string("Kevin Bacon"), string("Kevin Bacon")));
    CHECK(reachableDFS(graph, string("Ace Watson"), string("Ace Watson")));
    delete graph;
}

TEST(grading_dfs_socialNetworkDisconnected_20) {
    Graph<string, int>* graph = readGraphFile<string,int>(
        "test_data/socialNetworkDisconnected_20.graph");
    CHECK(reachableDFS(graph, string("Kevin Bacon"), string("Pansy Stele")));
    CHECK(reachableDFS(graph, string("Alexander Danseur"), string("Nanny Summer")));
    CHECK(reachableDFS(graph, string("Abner Corpus"), string("Toot Bacon")));
    CHECK(!reachableDFS(graph, string("Toot Bacon"), string("Toby Hog")));
    CHECK(!reachableDFS(graph, string("Edessa Pork"), string("Nanny Summer")));
    CHECK(!reachableDFS(graph, string("Abner Corpus"), string("Alexander Danseur")));
    delete graph;
}

TEST(bfsThreeVertexClique) {
    // This call makes an undirected graph.
    Graph<string, int>* graph =
        makeGraph<string, int>({"1", "2", "3"}, {
                                                    {"1", "2", 1},
                                                    {"1", "3", 1},
                                                    {"2", "3", 1},
                                                });
    vector<string> path = {"1", "3"};
    checkEqualPath(path,
                   shortestLengthPathBFS(graph, string("1"), string("3")));
    path = {"1", "2"};
    checkEqualPath(path,
                   shortestLengthPathBFS(graph, string("1"), string("2")));
    delete graph;
}

TEST(bfsFiveVertex) {
    // This call makes an undirected graph.
    Graph<string, int>* graph = makeGraph<string, int>(
        {"start", "1", "2", "finish", "alt"}, {{"start", "1", 1},
                                               {"1", "2", 1},
                                               {"2", "finish", 1},
                                               {"start", "alt", 1},
                                               {"alt", "finish", 1}});
    vector<string> path = {"start", "alt", "finish"};
    checkEqualPath(
        path, shortestLengthPathBFS(graph, string("start"), string("finish")));
    delete graph;
}

TEST(bfsFiveVertexDirected) {
    // This call makes a directed graph.
    Graph<string, int>* graph =
        makeGraph<string, int>(
            {"start", "1", "2", "finish", "alt"}, {{"start", "1", 1},
                                                   {"1", "2", 1},
                                                   {"2", "finish", 1},
                                                   {"finish", "alt", 1},
                                                   {"alt", "start", 1}},
                                                   true);
    vector<string> path = {"start", "1", "2", "finish"};
    checkEqualPath(
        path, shortestLengthPathBFS(graph, string("start"), string("finish")));
    delete graph;
}

TEST(grading_bfs_socialNetwork_2) {
    Graph<string, int>* graph = readGraphFile<string,int>(
        "test_data/socialNetwork_2.graph");
    vector<string> path;
    path = {"Kevin Bacon", "Gryllus Floyd"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Kevin Bacon"), string("Gryllus Floyd")));
    path = {"Kevin Bacon"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Kevin Bacon"), string("Kevin Bacon")));
    path = {"Gryllus Floyd"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Gryllus Floyd"), string("Gryllus Floyd")));
    path = {"Gryllus Floyd", "Kevin Bacon"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Gryllus Floyd"), string("Kevin Bacon")));
    delete graph;
}

TEST(grading_bfs_socialNetwork_4) {
    Graph<string, int>* graph = readGraphFile<string,int>(
        "test_data/socialNetwork_4.graph");
    vector<string> path;
    path = {"Gryllus Floyd"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Gryllus Floyd"), string("Gryllus Floyd")));
    path = {"Gryllus Floyd", "Kevin Bacon"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Gryllus Floyd"), string("Kevin Bacon")));
    path = {"Jared Watson"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Jared Watson"), string("Jared Watson")));
    CHECK_THROW(shortestLengthPathBFS(graph, string("Gryllus Floyd"), string("Jared Watson")), std::exception);
    delete graph;
}

TEST(grading_bfs_socialNetwork_8) {
    Graph<string, int>* graph = readGraphFile<string,int>(
        "test_data/socialNetwork_8.graph");
    vector<string> path;
    path = {"Gryllus Floyd", "Jaimie Dengar"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Gryllus Floyd"), string("Jaimie Dengar")));
    path = {"Oolong Danseur", "Kevin Bacon", "Ace Watson"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Oolong Danseur"), string("Ace Watson")));
    path = {"Wibbly Bajie", "Pink Pigoon", "Kevin Bacon", "Gryllus Floyd"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Wibbly Bajie"), string("Gryllus Floyd")));
    path = {"Kevin Bacon"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Kevin Bacon"), string("Kevin Bacon")));
    path = {"Kevin Bacon", "Ace Watson"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Kevin Bacon"), string("Ace Watson")));
    delete graph;
}

TEST(grading_bfs_socialNetwork_25) {
    Graph<string, int>* graph = readGraphFile<string,int>(
        "test_data/socialNetwork_25.graph");
    vector<string> path;
    path = {"Hamlet Hamilton", "Fener Trwyth", "Gryllus Floyd", "Oolong Danseur"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Hamlet Hamilton"), string("Oolong Danseur")));
    path = {"Waddles Hog", "Grunty Dengar", "Ace Watson", "Jared Watson"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Waddles Hog"), string("Jared Watson")));
    path = {"Gryllus Floyd", "Pink Pigoon"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Gryllus Floyd"), string("Pink Pigoon")));
    path = {"Hogzilla Corpus", "Fener Trwyth"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Hogzilla Corpus"), string("Fener Trwyth")));
    path = {"Edessa Porker", "Ace Watson", "Hogzilla Corpus"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Edessa Porker"), string("Hogzilla Corpus")));
    delete graph;
}

TEST(grading_bfs_socialNetwork_100) {
    Graph<string, int>* graph = readGraphFile<string,int>(
        "test_data/socialNetwork_100.graph");
    vector<string> path;
    path = {"Spot Wen", "Jaimie Dengar", "Gryllus Floyd"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Spot Wen"), string("Gryllus Floyd")));
    path = {"Freddy Boar", "Karnac Wen", "Napoleon Strangepork"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Freddy Boar"), string("Napoleon Strangepork")));
    path = {"Freddy Pigasus", "Erymanthian Dorf", "Hamlet Rosso", "Cicero Pettitoes", "Kevin Bacon", "Chuck Danseur"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Freddy Pigasus"), string("Chuck Danseur")));
    path = {"Hamlet Rosso", "Cicero Pettitoes", "Karnac Wen", "Karnac Hog", "Butasan Bajie"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Hamlet Rosso"), string("Butasan Bajie")));
    path = {"Squigley Pigoon", "Karnac Wen", "Napoleon Strangepork", "Link Poppleton"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Squigley Pigoon"), string("Link Poppleton")));
    path = {"Josephine Ham", "Kevin Bacon", "Freddy Boar", "Peppa Pig"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Josephine Ham"), string("Peppa Pig")));
    path = {"Jaimie Dengar", "Karnac Wen", "Brad Trwyth"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Jaimie Dengar"), string("Brad Trwyth")));
    path = {"Positive Robinson", "Jaimie Dengar", "Karnac Wen", "Napoleon Strangepork", "Peking Saddlebottom"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Positive Robinson"), string("Peking Saddlebottom")));
    delete graph;
}

TEST(grading_bfs_socialNetworkDisconnected_4) {
    Graph<string, int>* graph = readGraphFile<string,int>(
        "test_data/socialNetworkDisconnected_4.graph");
    vector<string> path;
    CHECK_THROW(shortestLengthPathBFS(graph, string("Kevin Bacon"), string("Ace Watson")), std::exception);
    CHECK_THROW(shortestLengthPathBFS(graph, string("Ace Watson"), string("Gryllus Floyd")), std::exception);
    CHECK_THROW(shortestLengthPathBFS(graph, string("Gryllus Floyd"), string("Jared Watson")), std::exception);
    CHECK_THROW(shortestLengthPathBFS(graph, string("Gryllus Floyd"), string("Ace Watson")), std::exception);
    delete graph;
}

TEST(grading_bfs_socialNetworkDisconnected_20) {
    Graph<string, int>* graph = readGraphFile<string,int>(
        "test_data/socialNetworkDisconnected_20.graph");
    vector<string> path;
    path = {"Kevin Bacon", "Pigby Strangepork"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Kevin Bacon"), string("Pigby Strangepork")));
    CHECK_THROW(shortestLengthPathBFS(graph, string("Rooter Ziffel"), string("Cross-patch Iron")), std::exception);
    path = {"Toby Hog", "Kevin Bacon", "Alexander Danseur"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Toby Hog"), string("Alexander Danseur")));
    CHECK_THROW(shortestLengthPathBFS(graph, string("Zhu Bland"), string("Toot Piggy")), std::exception);
    path = {"Toot Piggy", "Pansy Stele"};
    checkShortestPath(graph, path, shortestLengthPathBFS(graph, string("Toot Piggy"), string("Pansy Stele")));
    CHECK_THROW(shortestLengthPathBFS(graph, string("Pansy Stele"), string("Toot Bacon")), std::exception);
    delete graph;
}

TEST(ssspFiveVertex) {
    // This call makes an undirected graph.
    Graph<string, int>* graph = makeGraph<string, int>(
        {"start", "1", "2", "finish", "alt"}, {{"start", "1", 3},
                                               {"1", "2", 4},
                                               {"2", "finish", 2},
                                               {"start", "alt", 5},
                                               {"alt", "finish", 6}});
    Dictionary<string, int>* results =
        singleSourceShortestPath(graph, string("start"));
    CHECK_EQUAL(9, results->get("finish"));
    CHECK_EQUAL(7, results->get("2"));
    CHECK_EQUAL(5, results->get("alt"));
    CHECK_EQUAL(5, results->getSize());
    delete results;
    delete graph;
}

TEST(ssspFiveVertexDirected) {
    // This call makes a directed graph.
    Graph<string, int>* graph =
        makeGraph<string, int>(
            {"start", "1", "2", "finish", "alt"},
            {{"finish", "2", 3},
             {"2", "1", 4},
             {"1", "start", 2},
             {"start", "alt", 5},
             {"alt", "finish", 6}},
             true);
    Dictionary<string, int>* results =
        singleSourceShortestPath(graph, string("start"));
    CHECK_EQUAL(11, results->get("finish"));
    CHECK_EQUAL(14, results->get("2"));
    CHECK_EQUAL(5, results->get("alt"));
    CHECK_EQUAL(5, results->getSize());
    delete results;
    delete graph;
}

TEST(ssspFiveVertexDirectedUnreachable) {
    // This call makes a directed graph.
    Graph<string, int>* graph =
        makeGraph<string, int>(
            {"start", "1", "2", "finish", "alt"},
            {{"finish", "2", 3},
             {"1", "start", 2},
             {"start", "alt", 5},
             {"alt", "finish", 6}},
             true);
    Dictionary<string, int>* results =
        singleSourceShortestPath(graph, string("start"));

    CHECK_EQUAL(4, results->getSize());
    CHECK_EQUAL(11, results->get("finish"));
    CHECK_EQUAL(14, results->get("2"));
    CHECK_EQUAL(5, results->get("alt"));
    CHECK(!results->contains("1"));
    delete results;
    delete graph;
}

TEST(ssspTown8) {
    // Creates an undirected graph
    Graph<string, float>* graph = readGraphFile<string, float>(
        "test_data/town_8.graph");
    Dictionary<string, float>* results;
    results = singleSourceShortestPath(graph, string("Jimmy's Spa City"));
    CHECK_CLOSE(2.00, results->get("Pigma's Jewelry Museum"), 0.001);
    CHECK_CLOSE(3.00, results->get("Gaston's Office Supply Warehouse"), 0.001);
    CHECK_CLOSE(4.15, results->get("Cerdotado's Candy City"), 0.001);
    CHECK_CLOSE(7.50, results->get("Pigling Bland's Ice Cream Shack"), 0.001);
    delete results;
    results = singleSourceShortestPath(graph, string("Arnold's Convenience Stand"));
    CHECK_CLOSE(13.00, results->get("Cerdotado's Candy City"), 0.001);
    CHECK_CLOSE(25.66, results->get("Sam's Photo Retailer"), 0.001);
    CHECK_CLOSE(13.93, results->get("Jimmy's Spa City"), 0.001);
    CHECK_CLOSE(14.93, results->get("Petunia's Bowling Town"), 0.001);
    delete results;
    results = singleSourceShortestPath(graph, string("Sam's Photo Retailer"));
    CHECK_CLOSE(24.31, results->get("Pigling Bland's Ice Cream Shack"), 0.001);
    CHECK_CLOSE(0.00, results->get("Sam's Photo Retailer"), 0.001);
    CHECK_CLOSE(18.17, results->get("Gaston's Office Supply Warehouse"), 0.001);
    CHECK_CLOSE(17.81, results->get("Petunia's Bowling Town"), 0.001);
    delete results;
    delete graph;
}

TEST(ssspTown100) {
    // Creates an undirected graph
    Graph<string, float>* graph = readGraphFile<string, float>(
        "test_data/town_100.graph");
    Dictionary<string, float>* results;
    results = singleSourceShortestPath(graph, string("Saddlebottom's Photo Trough"));
    CHECK_CLOSE(19.65, results->get("Snorty's Fuel Stand"), 0.001);
    CHECK_CLOSE(20.99, results->get("Huxley's Furniture City"), 0.001);
    CHECK_CLOSE(24.81, results->get("Josephine's Furniture Ltd."), 0.001);
    CHECK_CLOSE(27.85, results->get("Hogthrob's Hat Museum"), 0.001);
    CHECK_CLOSE(35.04, results->get("Boar's Electronics Trough"), 0.001);
    CHECK_CLOSE(21.62, results->get("Pequeninos's Produce Ltd."), 0.001);
    delete results;
    results = singleSourceShortestPath(graph, string("Ace's Antique Store"));
    CHECK_CLOSE(29.36, results->get("Arbuckle's Blanket Emporkium"), 0.001);
    CHECK_CLOSE(26.25, results->get("Petunia's Antique Ltd."), 0.001);
    CHECK_CLOSE(37.31, results->get("Boar's Electronics Trough"), 0.001);
    CHECK_CLOSE(37.16, results->get("Summer's Bargain Vault"), 0.001);
    CHECK_CLOSE(21.85, results->get("Josephine's Gaming Museum"), 0.001);
    CHECK_CLOSE(32.18, results->get("Pigma's Candy Barn"), 0.001);
    delete results;
    results = singleSourceShortestPath(graph, string("Huxley's Furniture City"));
    CHECK_CLOSE(36.75, results->get("Percy's Blanket Vault"), 0.001);
    CHECK_CLOSE(23.38, results->get("Arbuckle's Blanket Emporkium"), 0.001);
    CHECK_CLOSE(28.45, results->get("Toot's Clothing Vault"), 0.001);
    CHECK_CLOSE(10.79, results->get("Gaston's Office Supply Warehouse"), 0.001);
    CHECK_CLOSE(9.79, results->get("Pigma's Jewelry Museum"), 0.001);
    CHECK_CLOSE(0.00, results->get("Huxley's Furniture City"), 0.001);
    delete results;
    results = singleSourceShortestPath(graph, string("Josephine's Gaming Museum"));
    CHECK_CLOSE(13.12, results->get("Poppleton's Literature Retailer"), 0.001);
    CHECK_CLOSE(37.51, results->get("Dean's Jewelry Trough"), 0.001);
    CHECK_CLOSE(25.80, results->get("Josephine's Furniture Ltd."), 0.001);
    CHECK_CLOSE(19.23, results->get("Ziggy's Bowling Hut"), 0.001);
    CHECK_CLOSE(35.58, results->get("Akio's Pizza Acres"), 0.001);
    CHECK_CLOSE(22.39, results->get("Gryllus's Gaming Trough"), 0.001);
    delete results;
    delete graph;
}

int main() {
    return UnitTest::RunAllTests();
}

void checkEqualPath(vector<string> expected, vector<string> actual) {
    string expectedPath = "";
    for (int i = 0; i < expected.size(); i++) {
        if (expectedPath.size() > 0) {
            expectedPath += " -- ";
        }
        expectedPath += expected.at(i);
    }
    string actualPath = "";
    for (int i = 0; i < actual.size(); i++) {
        if (actualPath.size() > 0) {
            actualPath += " -- ";
        }
        actualPath += actual.at(i);
    }
    CHECK_EQUAL("path = " + expectedPath, "path = " + actualPath);
}

template <typename T> void noDupesCheck(std::vector<T> expr) {
    auto vec = expr;
    auto it = vec.begin();
    while (it < vec.end()) {
        if (std::find(it + 1, vec.end(), *it) != vec.end()) {
            CHECK_EQUAL(*it + " to be unique in vector",
                        *it + " in vector twice");
        }
        it++;
    }
}

template <typename T>
void vectorsEqualCheck(std::vector<T> first, std::vector<T> second) {
    auto expected = first;
    auto actual = second;
    noDupesCheck(actual);
    CHECK_EQUAL(expected.size(), actual.size());
    for (int i = 0; i < expected.size(); i++) {
        if (std::find(actual.begin(), actual.end(), expected[i]) ==
            actual.end()) {
            CHECK_EQUAL(expected[i] + " to be in vector", "no such result");
        }
    }
    for (int i = 0; i < actual.size(); i++) {
        if (std::find(expected.begin(), expected.end(), actual[i]) ==
            expected.end()) {
            CHECK_EQUAL(actual[i] + " not to be in vector",
                        actual[i] + " in vector");
        }
    }
}

template <typename W>
void checkShortestPath(Graph<string, W>* graph, vector<string> expected, vector<string> actual) {
    if (expected.size() != actual.size()) {
        CHECK_EQUAL(string("Expected path length: ") + to_string(expected.size()),
                    string("Actual path length: ") + to_string(actual.size()));
        return;
    }
    for (int i=0;i<actual.size()-1;i++) {
        if (!graph->containsEdge(actual[i], actual[i+1])) {
            CHECK_EQUAL(string("path to contain only edges in the graph"),
                        string("a path with edge ") +
                        actual[i] + " -> " + actual[i+1]);
        }
    }
}
