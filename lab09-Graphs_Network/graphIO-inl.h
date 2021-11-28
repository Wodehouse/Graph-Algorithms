#include "adjacencyListGraph.h"

/**
 * Reads a type from a file via a function call rather than the >> operator.
 * This allows for project-specific customization.
 * @param file The file from which to read.
 * @return The data read from the file.
 */
template <typename T>
inline T graphRead(std::ifstream& file) {
    T data;
    file >> data;
    return data;
}

/**
 * A specialized interface for reading strings from a file.  We assume here that
 * strings are terminated with newlines.  We begin by skipping whitespace (as
 * ifstream does on >> by default) and then construct the string out of each
 * character we find until encountering a newline.
 * @param file The file from which to read the string.
 * @return The string read from the file.
 */
template<>
inline std::string graphRead<std::string>(std::ifstream& file) {
    using namespace std; // you can do this inside of functions!
    int c = file.peek();
    while (c == ' ' || c == '\n' || c == '\t' || c == '\r') {
        file.get();
        c = file.peek();
    }
    string result = "";
    while (c != EOF && c != '\n') {
        result += (char)c;
        file.get();
        c = file.peek();
    }
    if (c == '\n') {
        file.get();
    }
    return result;
}

template <typename V, typename W>
Graph<V,W>* loadGraph(std::string filename, bool directed) {
    Graph<V,W>* graph = new AdjacencyListGraph<V,W>();
    using namespace std; // you can do this inside of functions!
    ifstream file(filename.c_str());
    // Make the file throw exceptions if something goes wrong.
    file.exceptions( std::ifstream::failbit | std::ifstream::badbit );
    string block_type;
    file >> block_type;
    while (block_type != "end") {
        if (block_type == "vertex") {
            V vertex = graphRead<V>(file);
            graph->insertVertex(vertex);
        } else if (block_type == "edge") {
            V source = graphRead<V>(file);
            V target = graphRead<V>(file);
            W weight = graphRead<W>(file);
            graph->insertEdge(source, target, weight);
            if (!directed && !graph->containsEdge(target, source)) {
                graph->insertEdge(target, source, weight);
            }
        } else {
            throw runtime_error("Unrecognized block type: " + block_type);
        }
        file >> block_type;
    }
    return graph;
}
