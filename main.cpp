#include <iostream>
#include <vector>
#include <queue>
#include <climits>
using namespace std;

class Graph {
private:
    int V; // Number of vertices
    vector<vector<pair<int, int>>> adj; // Adjacency list: pair(destination, weight)
    
    // Helper function for DFS
    void DFSUtil(int node, vector<bool>& visited) {
        visited[node] = true;
        cout << node << " ";
        
        for (auto& neighbor : adj[node]) {
            int dest = neighbor.first;
            if (!visited[dest]) {
                cout << "\n  Going deeper to neighbor " << dest << endl;
                DFSUtil(dest, visited);
            }
        }
    }

public:
    // Constructor
    Graph(int vertices) {
        V = vertices;
        adj.resize(V);
    }
    
    // Add weighted directed edge
    void addEdge(int u, int v, int weight) {
        adj[u].push_back(make_pair(v, weight));
    }
    
    // Display graph structure
    void displayGraph() {
        cout << "\n=== Graph Structure (Adjacency List) ===\n";
        for (int i = 0; i < V; i++) {
            cout << "Vertex " << i << ":";
            for (auto& edge : adj[i]) {
                cout << " -> (" << edge.first << ", " << edge.second << ")";
            }
            cout << endl;
        }
        cout << endl;
    }
    
    // Breadth-First Search
    void BFS(int start) {
        if (start < 0 || start >= V) {
            cout << "Invalid starting node!\n";
            return;
        }
        
        vector<bool> visited(V, false);
        queue<int> q;
        
        visited[start] = true;
        q.push(start);
        
        cout << "\n=== BFS Traversal ===\n";
        cout << "Starting BFS from node " << start << "\n\n";
        
        while (!q.empty()) {
            int node = q.front();
            q.pop();
            cout << "Visited: " << node << endl;
            
            for (auto& neighbor : adj[node]) {
                int dest = neighbor.first;
                if (!visited[dest]) {
                    visited[dest] = true;
                    q.push(dest);
                    cout << "  Exploring neighbor " << dest << endl;
                }
            }
        }
        cout << "\nBFS Complete.\n";
    }
    
    // Depth-First Search
    void DFS(int start) {
        if (start < 0 || start >= V) {
            cout << "Invalid starting node!\n";
            return;
        }
        
        vector<bool> visited(V, false);
        
        cout << "\n=== DFS Traversal ===\n";
        cout << "Starting DFS from node " << start << "\n\n";
        cout << "Visited: ";
        
        DFSUtil(start, visited);
        
        cout << "\n\nDFS Complete.\n";
    }
    
    // Dijkstra's Shortest Path Algorithm
    void Dijkstra(int start) {
        if (start < 0 || start >= V) {
            cout << "Invalid starting node!\n";
            return;
        }
        
        vector<int> dist(V, INT_MAX);
        vector<int> parent(V, -1);
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
        
        dist[start] = 0;
        pq.push(make_pair(0, start));
        
        cout << "\n=== Dijkstra's Shortest Path Algorithm ===\n";
        cout << "Starting from node " << start << "\n\n";
        
        while (!pq.empty()) {
            int u = pq.top().second;
            int currentDist = pq.top().first;
            pq.pop();
            
            if (currentDist > dist[u]) continue;
            
            cout << "Processing node " << u << " (distance: " << dist[u] << ")\n";
            
            for (auto& neighbor : adj[u]) {
                int v = neighbor.first;
                int weight = neighbor.second;
                
                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    parent[v] = u;
                    pq.push(make_pair(dist[v], v));
                    cout << "  Updated distance to node " << v << ": " << dist[v] << endl;
                }
            }
        }
        
        cout << "\n--- Final Shortest Distances from Node " << start << " ---\n";
        for (int i = 0; i < V; i++) {
            cout << "Node " << i << ": ";
            if (dist[i] == INT_MAX)
                cout << "∞ (unreachable)";
            else
                cout << dist[i];
            cout << endl;
        }
        
        cout << "\nDijkstra's Algorithm Complete.\n";
    }
};

int main() {
    cout << "========================================\n";
    cout << "    GRAPH VISUALIZER TOOL\n";
    cout << "    BFS, DFS, and Dijkstra's Algorithm\n";
    cout << "========================================\n\n";
    
    int vertices, edges;
    cout << "Enter number of vertices: ";
    cin >> vertices;
    
    Graph graph(vertices);
    
    cout << "Enter number of edges: ";
    cin >> edges;
    
    cout << "\nEnter edges (source destination weight):\n";
    for (int i = 0; i < edges; i++) {
        int u, v, w;
        cout << "Edge " << (i + 1) << ": ";
        cin >> u >> v >> w;
        graph.addEdge(u, v, w);
    }
    
    graph.displayGraph();
    
    int choice;
    do {
        cout << "\n========================================\n";
        cout << "Select Algorithm:\n";
        cout << "1. Breadth-First Search (BFS)\n";
        cout << "2. Depth-First Search (DFS)\n";
        cout << "3. Dijkstra's Shortest Path\n";
        cout << "4. Exit\n";
        cout << "========================================\n";
        cout << "Enter your choice: ";
        cin >> choice;
        
        if (choice >= 1 && choice <= 3) {
            int startNode;
            cout << "Enter starting node: ";
            cin >> startNode;
            
            switch(choice) {
                case 1:
                    graph.BFS(startNode);
                    break;
                case 2:
                    graph.DFS(startNode);
                    break;
                case 3:
                    graph.Dijkstra(startNode);
                    break;
            }
        }
        
    } while (choice != 4);
    
    cout << "\nThank you for using Graph Visualizer Tool!\n";
    return 0;
}
