#include <bits/stdc++.h>
using namespace std;

// Graph class representing BFS, DFS and Dijkstra's visualization
class Graph {
    int V;
    vector<vector<pair<int, int>>> adj; // adjacency list: (neighbor, weight)

public:
    Graph(int vertices) {
        V = vertices;
        adj.resize(V);
    }

    void addEdge(int u, int v, int w = 1) {
        adj[u].push_back({v, w});
        adj[v].push_back({u, w}); // undirected
    }

    void displayGraph() {
        cout << "\nGraph adjacency list:\n";
        for (int i = 0; i < V; ++i) {
            cout << i << " -> ";
            for (auto [nbr, w] : adj[i])
                cout << "(" << nbr << ", w=" << w << ") ";
            cout << "\n";
        }
    }

    void BFS(int start) {
        vector<bool> visited(V, false);
        queue<int> q;
        q.push(start);
        visited[start] = true;

        cout << "\nBFS Traversal (starting from " << start << "): ";
        while (!q.empty()) {
            int node = q.front();
            q.pop();
            cout << node << " ";
            for (auto [nbr, _] : adj[node]) {
                if (!visited[nbr]) {
                    visited[nbr] = true;
                    q.push(nbr);
                }
            }
        }
        cout << endl;
    }

    void DFSUtil(int node, vector<bool> &visited) {
        visited[node] = true;
        cout << node << " ";
        for (auto [nbr, _] : adj[node]) {
            if (!visited[nbr])
                DFSUtil(nbr, visited);
        }
    }

    void DFS(int start) {
        vector<bool> visited(V, false);
        cout << "\nDFS Traversal (starting from " << start << "): ";
        DFSUtil(start, visited);
        cout << endl;
    }

    void Dijkstra(int start) {
        vector<int> dist(V, INT_MAX);
        dist[start] = 0;
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;
        pq.push({0, start});

        cout << "\nDijkstra’s shortest paths from node " << start << ":\n";
        while (!pq.empty()) {
            int node = pq.top().second;
            int d = pq.top().first;
            pq.pop();

            for (auto [nbr, w] : adj[node]) {
                if (d + w < dist[nbr]) {
                    dist[nbr] = d + w;
                    pq.push({dist[nbr], nbr});
                }
            }
        }

        for (int i = 0; i < V; ++i)
            cout << "Distance to " << i << " = " << dist[i] << endl;
    }
};

int main() {
    int V, E;
    cout << "Enter number of vertices: ";
    cin >> V;
    Graph g(V);

    cout << "Enter number of edges: ";
    cin >> E;
    cout << "Enter edges (u v w):\n";
    for (int i = 0; i < E; ++i) {
        int u, v, w;
        cin >> u >> v >> w;
        g.addEdge(u, v, w);
    }

    g.displayGraph();

    int start;
    cout << "\nEnter start node for traversals: ";
    cin >> start;

    g.BFS(start);
    g.DFS(start);
    g.Dijkstra(start);

    cout << "\n✅ Visualization complete!" << endl;
    return 0;
}
