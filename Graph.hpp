#ifndef GRAPH_HPP
#define GRAPH_HPP

#include "GraphBase.hpp"
#include <iterator>
#include <queue>
#include <limits>
#include <vector>
#include <deque>
#include <algorithm>
#include <string>

class Edge {
private:
    std::string edge_a; // main edge
    std::string edge_b; // adjacent edge
    unsigned long weight; // weight of the edge
protected:
public:

    // default settings
    Edge(std::string a = "", std::string b = "", unsigned long w = 0) {
        edge_a = a;
        edge_b = b;
        weight = w;
    }
    ~Edge() {} // destructor

    friend class Graph;
};

class Vertex{
private:
    std::string vertex;
    unsigned long minimum_distance;
    std::vector<std::string> shortest_path;
    bool node_seen; // checks whether a node has been seen
protected:
public:
    friend class Graph;
    Vertex() { vertex = ""; } // constructor
    ~Vertex() {} // destructor
};

class Graph : public GraphBase {
private:

    std::priority_queue<std::pair<unsigned long, std::string>,
            std::vector<std::pair<unsigned long, std::string>>,
            std::greater<std::pair<unsigned long, std::string>>> list_of_paths;

    std::deque<Vertex*> vertexes; // vertexes and distances
    std::deque<Edge*> edges; //  edges and their weight
    unsigned long shortest_distance = 0; // holds the shortest distance after each node visit
    int minimum_vertex_idx = 0; // acquires the node index with minimum distances
    std::string minimum_element = ""; // Gets the minimum element at that node

public:
    Graph() {}
    ~Graph();

    void check_str(std::string input_str1, std::string input_str2); //Checks two string inputs*/
    void minimum_distance(std::string startLabel); // put distances in the string*/

    void addVertex(std::string label);
    void removeVertex(std::string label);
    void addEdge(std::string label1, std::string label2, unsigned long weight);
    void removeEdge(std::string label1, std::string label2);
    unsigned long shortestPath(std::string startLabel, std::string endLabel, std::vector<std::string> &path);
};
#endif // !GRAPH_HPP
