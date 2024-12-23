#include "graph.h"


// ------------------
// Constructor
// ------------------
template <typename T>
Graph<T>::Graph(int v) : vertices(v)  // The initialization list sets 'vertices' to 'v'
{
    // We resize the adjacency list to have 'v' "lists," one for each vertex
    adjList.resize(v);

    // We resize the vertexData array to store data (of type T) for each vertex
    vertexData.resize(v);
}

// ------------------
// setVertexData
// ------------------
template <typename T>
void Graph<T>::setVertexData(int v, const T& data)
{
    // Only set if v is a valid index: 0 <= v < vertices
    if (v >= 0 && v < vertices)
    {
        vertexData[v] = data;
    }
    
}

// ------------------
// getVertexData
// ------------------
template <typename T>
T Graph<T>::getVertexData(int v) const
{
    // If valid index, return the stored data
    if (v >= 0 && v < vertices) 
    {
        return vertexData[v];
    }
    // Otherwise, return a default constructed object of type T (e.g., 0 for int, "" for string, etc....
    return T();
}

// ------------------
// addEdge
// ------------------
template <typename T>
void Graph<T>::addEdge(int src, int dest, double weight)
{
    // Check that both src and dest are valid..
    if (src >= 0 && src < vertices && dest >= 0 && dest < vertices)
    {
        /* We use make_pair to create the pair<int, double>
         This represents an edge to 'dest' with 'weight' */
        adjList[src].push_back(make_pair(dest, weight));

        adjList[dest].push_back(make_pair(src, weight));
    }
}

// ------------------
// dfsHelper (recursive DFS logic)
// ------------------
template <typename T>
void Graph<T>::dfsHelper(int v, vector<bool>& visited)
{
    // Mark the current vertex 'v' as visited
    visited[v] = true;

    // Print out what we visited..
    cout << "Visited: " << v << " (Data = " << vertexData[v] << ")" << endl;

    /* Loop through all neighbors of vertex 'v'
       'neighbor' is a pair<int, double>, so neighbor.first is the neighbor index */
    for (auto& neighbor : adjList[v]) 
    {
        int adjVertex = neighbor.first;

        // If we haven't visited that neighbor, recurse on it...
        if (!visited[adjVertex])
        
{
            dfsHelper(adjVertex, visited);
        }
    }
}

// ------------------
// DFS (public interface)
// ------------------
template <typename T>
void Graph<T>::DFS(int start)
{
    // If 'start' is out-of-range, we just stop
    if (start < 0 || start >= vertices)
    {
        cout << "Invalid start vertex.\n";
        return;  
    }

    // Keep track of which vertices have been visited here.., initialized to false
    vector<bool> visited(vertices, false);

    // Call our helper to do the recursive DFS
    dfsHelper(start, visited);
}

// ------------------
// BFS
// ------------------
template <typename T>
void Graph<T>::BFS(int start)
{
    // Basic validation of 'start' index
    if (start < 0 || start >= vertices)
    {
        cout << "Invalid start vertex.\n";
        return;
    }

    // Keep track of visited vertices..
    vector<bool> visited(vertices, false);

    
    queue<int> q;

    // Mark the start vertex as visited and enqueue it
    visited[start] = true;
    q.push(start);

    // While we still have vertices in the queue
    while (!q.empty())
    {
        // Grab the front of the queue
        int current = q.front();
        q.pop();  // Remove it from the queue

        // Process it (in BFS, this is where you'd do whatever operation you need)
        cout << "Visited: " << current << " (Data = " << vertexData[current] << ")" << endl;

        // Enqueue all unvisited neighbors
        for (auto& neighbor : adjList[current])
        {
            int adjVertex = neighbor.first;
            if (!visited[adjVertex])
            {
                visited[adjVertex] = true;
                q.push(adjVertex);
            }
        }
    }
}

// ------------------
// Dijkstra's Algorithm
// ------------------
template <typename T>
void Graph<T>::dijkstra(int start)
{
    // If invalid 'start', just exit the function.. 
    if (start < 0 || start >= vertices)
    {
        cout << "Invalid start vertex.\n";
        return;
    }

    /* 'dist' will hold the shortest distance from 'start' to each vertex. 
        Initialize all distances to infinity */
    vector<double> dist(vertices, numeric_limits<double>::infinity());

    // The distance from 'start' to itself is 0...
    dist[start] = 0.0;

    /* We use a min - heap(priority_queue) of(distance, vertex)
        The 'greater' comparator makes it behave like a min-heap */
    priority_queue< pair<double, int>,
        vector< pair<double, int> >,
        greater< pair<double, int> > > pq;

    /* Begin by pushing(0.0, start) into the priority queue
       meaning distance is 0.0 for the start vertex */
    pq.push(make_pair(0.0, start));

    // Keep going until there are no more vertices to process
    while (!pq.empty())
    {
        // Retrieve the pair with the smallest distance
        pair<double, int> topPair = pq.top();
        double currentDist = topPair.first;
        int u = topPair.second;

        // Remove it from the queue
        pq.pop();

        // If the recorded distance is larger than what we have in dist[u], skip it
        if (currentDist > dist[u])
        {
            continue;
        }

        // For each neighbor of 'u'
        for (auto& edge : adjList[u])
        {
            int v = edge.first;
            double weight = edge.second;

            // If going from 'u' to 'v' is better than our current best known distance, update it
            if (dist[u] + weight < dist[v])
            {
                dist[v] = dist[u] + weight;
                // Push the updated distance into the queue
                pq.push(make_pair(dist[v], v));
            }
        }
    }

    // Finally, print out the shortest distances from 'start' to each vertex...
    cout << "\nShortest distances from vertex " << start << ":\n";
    for (int i = 0; i < vertices; i++)
    {
        cout << "Vertex " << i << " (Data = " << vertexData[i]
            << ") Distance = " << dist[i] << endl;
    }
}

// ------------------
// Explicit Instantiations
// ------------------
template class Graph<int>;
template class Graph<string>;
