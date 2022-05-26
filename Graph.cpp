#include "Graph.hpp"

// Generates and adds a vertex to the graph with label
// two vertexes can not have the same label
void Graph::addVertex(std::string label)
{
    if (label.empty()) {
        throw label;
    }
    Vertex *new_vertex = new Vertex;

    // Set initial distance to infinite
    new_vertex->vertex = label;
    new_vertex->minimum_distance = std::numeric_limits<unsigned long>::max();
    new_vertex->shortest_path = { "" };
    new_vertex->node_seen = false;

    vertexes.push_back(new_vertex); // Push back in the container with vertex
}

// removes the vertex with label from the graph.
// removes the edges between that vertex and other vertices of the graph.
void Graph::removeVertex(std::string label)
{
    if (label.empty()) {
        throw label;
    }

    // Remove the vertex
    for (std::deque<Vertex*>::iterator ver_itr = vertexes.begin(); ver_itr != vertexes.end();) {
        if ((*ver_itr)->vertex == label) {
            vertexes.erase(ver_itr);
        }
        else {}
        ++ver_itr;
    }

    // Remove edges associated with that node
    for (std::deque<Edge*>::iterator edg_itr = edges.begin(); edg_itr != edges.end();) {
        if (((*edg_itr)->edge_a == label) || ((*edg_itr)->edge_b == label)) {
            edges.erase(edg_itr);
        }
        else{}
        ++edg_itr;
    }
}


// Throw if both string input are null
void Graph::check_str(std::string input_str1, std::string input_str2) {
    if (input_str1.empty()) {
        throw input_str1;
    }
    else if (input_str2.empty()) {
        throw input_str2;
    }
    else if (input_str1.empty() && input_str2.empty()) {
        throw ("String arguments contain are empty.\n");
    }
}

// Adds an edge of value weight to the graph between the vertex with label1 and the vertex with label2
void Graph::addEdge(std::string label1, std::string label2, unsigned long weight)
{
    check_str(label1, label2);
    // Add an edge in between label1 and label2
    Edge *newEdge1 = new Edge(label1, label2, weight);
    Edge* newEdge2 = new Edge(label2, label1, weight);

    edges.push_back(newEdge1);
    edges.push_back(newEdge2);
}


// Removes the edge from the graph between the vertex with label1 and the vertex with label
void Graph::removeEdge(std::string label1, std::string label2)
{
    check_str(label1, label2);
    // Only erase if both label match with the two label
    for (std::deque<Edge*>::iterator edg_itr = edges.begin(); edg_itr != edges.end();) {
        if (((*edg_itr)->edge_a == label1) && ((*edg_itr)->edge_b == label2)) {
            edges.erase(edg_itr);
        }
        else{}
        ++edg_itr;
    }
}


// Calculates the shortest path between the vertex with startLabel and the vertex with endLabel
// A vector is passed into the function that stores the shortest path between the vertices.
// The return value is the total sum of the edges between the start and end vertices on the shortest path.
unsigned long Graph::shortestPath(std::string startLabel, std::string endLabel, std::vector<std::string>& path)
{
    check_str(startLabel, endLabel);

    for (std::deque<Vertex*>::iterator ver_itr = vertexes.begin(); ver_itr != vertexes.end();) {
        if ((*ver_itr)->vertex == startLabel) {
            (*ver_itr)->minimum_distance = 0;
            (*ver_itr)->shortest_path.clear();
            (*ver_itr)->shortest_path.push_back(startLabel);
        }
        else{}
        ++ver_itr;
    }
    list_of_paths.push(std::make_pair(0, startLabel));

    // visiting all possible paths
    for (;!list_of_paths.empty();) {

        minimum_element = list_of_paths.top().second;

        list_of_paths.pop();

        int ver_idx = 0;
        for (std::deque<Vertex*>::iterator ver_itr = vertexes.begin(); ver_itr != vertexes.end();) {
            if ((*ver_itr)->vertex == minimum_element) {
                minimum_vertex_idx = ver_idx;
            }
            else{ ++ver_idx;}
            ++ver_itr;
        }

        minimum_distance(startLabel); // Get the weight

        vertexes.at(minimum_vertex_idx)->node_seen = true;
    }

    for (std::deque<Vertex*>::iterator ver_itr = vertexes.begin(); ver_itr != vertexes.end();) {
        if (endLabel == (*ver_itr)->vertex) { // Loop until find the endLabel
            shortest_distance = (*ver_itr)->minimum_distance; // Get the minimum distaces and store
            (*ver_itr)->shortest_path.push_back(endLabel); // Put the least weight in the string
            path = (*ver_itr)->shortest_path; // store that minimum path into vector of path and continue
        }
        else{}
        ++ver_itr;
    }

    // Clear the vertex container
    for (std::deque<Vertex*>::iterator ver_itr = vertexes.begin(); ver_itr != vertexes.end();) {
        (*ver_itr)->minimum_distance = std::numeric_limits<unsigned long>::max();;
        (*ver_itr)->shortest_path = { "" };
        (*ver_itr)->node_seen = false;
        ++ver_itr;
    }

    return shortest_distance; // Return the shortest distance and we are done
}


// Using BFS, traverse through edge and vertex containers and push possible paths into the priority queue
void Graph::minimum_distance(std::string startLabel){
    // Traverse through edge
    for (std::deque<Edge*>::iterator edg_itr = edges.begin(); edg_itr != edges.end();) {

        if ((*edg_itr)->edge_a == minimum_element) { // If one edge toward B matches the minimum element

            // Start iterating over vertex adjacent to that vector from above
            for (std::deque<Vertex*>::iterator ver_itr = vertexes.begin(); ver_itr != vertexes.end();) {
                // If going backward has a smaller weight and has been not visited
                if (((*edg_itr)->edge_b == (*ver_itr)->vertex) &&
                    ((vertexes.at(minimum_vertex_idx)->minimum_distance + (*edg_itr)->weight) < (*ver_itr)->minimum_distance) &&
                    ((*ver_itr)->node_seen == false))
                { // Store the distance weight
                    (*ver_itr)->minimum_distance = vertexes.at(minimum_vertex_idx)->minimum_distance + (*edg_itr)->weight;

                    if (minimum_element == startLabel) {
                        (*ver_itr)->shortest_path.clear(); // Clear the path, and push A
                        (*ver_itr)->shortest_path.push_back(startLabel);
                    }
                    else if (minimum_element != startLabel) { // push a node/vertex at the minimum index
                        (*ver_itr)->shortest_path.clear();
                        (*ver_itr)->shortest_path = vertexes.at(minimum_vertex_idx)->shortest_path;
                        (*ver_itr)->shortest_path.push_back(vertexes.at(minimum_vertex_idx)->vertex);
                    }
                    // Make a pair and push into the priority queue
                    list_of_paths.push(std::make_pair((*ver_itr)->minimum_distance, (*ver_itr)->vertex));
                }
                else{}
                ++ver_itr;
            }
        }
        else{}
        ++edg_itr;
    }
}

// Destructor
Graph::~Graph()
{
    for(auto& x : vertexes) { delete x; }
    for(auto& x : edges) { delete x; }
    vertexes.clear();
    edges.clear();
}

