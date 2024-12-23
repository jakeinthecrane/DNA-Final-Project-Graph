#include "graph.h"

int main() 
{
    // Create a Graph of 5 vertices, storing int data
    Graph<int> g(5);  // g is an instance of a user defined class template named Graph  (we can also refer to a vertice as a node but we are discussing graphs here so we stay on topic)

    // Set some data on each vertex (0 through 4)
    for (int i = 0; i < 5; i++)
    {
        g.setVertexData(i, i * 10);  // e.g., vertex i has data i*10
    }

    // Add edges (undirected)
    g.addEdge(0, 1, 2.0);  // we call the addEdge function with same graph object with a separate edge connection between 2 specified vertices on the same graph.  in this case vertex 0 and vertex 1 with a weight of 2.0 
    g.addEdge(0, 2, 4.0);  // vertex 0 and vertex 2 with a weight of 4.0 and so forth for the next 3... 
    g.addEdge(1, 3, 3.0);
    g.addEdge(2, 3, 1.0);
    g.addEdge(3, 4, 5.0);
     // after these 5 calls we'll have 5 edges  connecting 5 vertices in different ways.  We can now run the following 3 graph algorithms on that graph...

    // here is 3 different approaches to the same graph structure...

    cout << "=== DFS from vertex 0 ===" << endl;
    g.DFS(0); // Depth-First Stretch starting at vertex 0; dfs explores as far as possible along a branch before backtracking.  

    cout << "\n=== BFS from vertex 0 ===" << endl;
    g.BFS(0); // Breadth-First Stretch starting at vertex 0; bfs explores neighbors first before moving outward to more distant vertices. 

    cout << "\n=== Dijkstra from vertex 0 ===" << endl;
    g.dijkstra(0); // dijkstra computes the shortest paths (minimum total weight) from vertex 0 to all other vertices in the graph.  



    return 0;
}
