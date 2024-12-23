#pragma once
#include <iostream>
#include <vector>
#include <list>
#include <queue>
#include <stack>
#include <limits>   // For numeric_limits
#include <utility>  // For pair
#include <algorithm> // For std::make_heap, etc.

using namespace std;

template <typename T>
class Graph 
{
private:
    // Number of vertices
    int vertices;
    // Adjacency list: each element is a list of (neighbor, weight)
    vector< list< pair<int, double> > > adjList;
    // (Optional) Store a label or data for each vertex
    vector<T> vertexData;

    // Helper function for recursive DFS
    void dfsHelper(int v, vector<bool>& visited);

public:
    // Constructor
    Graph(int v);

    // Set data for a given vertex
    void setVertexData(int v, const T& data);

    // Get data for a given vertex
    T getVertexData(int v) const;

    // Add an edge (undirected for simplicity, can adapt for directed)
    void addEdge(int src, int dest, double weight = 1.0);

    // Depth-First Search (DFS) - uses recursion
    void DFS(int start);

    // Breadth-First Search (BFS)
    void BFS(int start);

    // Dijkstra’s Algorithm
    void dijkstra(int start);
};
