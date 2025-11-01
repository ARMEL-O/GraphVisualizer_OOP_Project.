# 🧭 Graph Visualizer Tool  
**Visual Representation of BFS, DFS, and Dijkstra’s Algorithm**

---

## 📘 Overview
The **Graph Visualizer Tool** is a **C++ console-based application** designed to help students and developers understand how graph algorithms such as **Breadth-First Search (BFS)**, **Depth-First Search (DFS)**, and **Dijkstra’s Shortest Path Algorithm** work internally.  
It provides a **step-by-step textual visualization** of node traversal, distance updates, and algorithm progression.

---

## 🎯 Objectives
- Implement and demonstrate **BFS**, **DFS**, and **Dijkstra’s Algorithm** using **C++**.
- Visualize traversal and pathfinding in a **text-based format**.
- Strengthen understanding of **graph theory** and **OOP principles**.
- Create a **reusable**, **modular**, and **extensible** codebase for educational use.

---

## 🧩 Features
✅ Console-based visualization of graph traversals  
✅ Supports weighted, undirected graphs  
✅ Demonstrates BFS, DFS, and Dijkstra step-by-step  
✅ Simple interface for educational purposes  
✅ Modular **OOP design** (encapsulation and reusability)

---

## 🛠️ Tools & Technologies Used
| Technology | Description |
|-------------|-------------|
| **C++ 20** | Core programming language used |
| **STL (Standard Template Library)** | Provides `vector`, `queue`, and `priority_queue` |
| **OnlineGDB Compiler** | Online IDE used for development and testing |
| **Console I/O** | For text-based visualization |
| **OOP Concepts** | Used for modular and clean design |

---

## 🧱 System Design

### **Class Diagram**

| **Class Name** | **Graph** |
|-----------------|-----------|
| **Attributes (Private)** | **Type** |
| `V` | `int` |
| `adj` | `vector<vector<pair<int, int>>>` |
| **Methods (Public)** | **Description** |
| `Graph(int vertices)` | Constructor to initialize the graph |
| `addEdge(int u, int v, int w)` | Adds an edge between vertices |
| `displayGraph()` | Displays adjacency list |
| `BFS(int start)` | Performs Breadth-First Search |
| `DFS(int start)` | Performs Depth-First Search |
| `Dijkstra(int start)` | Finds shortest paths using Dijkstra’s algorithm |
| **Helper (Private)** | **Description** |
| `DFSUtil(int node, vector<bool>& visited)` | Recursive DFS helper |

---

## ⚙️ Algorithms Implemented

### **1. Breadth-First Search (BFS)**
- Traverses nodes level-by-level.
- Uses a **queue (FIFO)**.
- **Time Complexity:** O(V + E)
- **Applications:** Web crawling, shortest path in unweighted graphs.

### **2. Depth-First Search (DFS)**
- Explores deeply before backtracking.
- Uses **recursion or stack**.
- **Time Complexity:** O(V + E)
- **Applications:** Cycle detection, pathfinding, topological sorting.

### **3. Dijkstra’s Algorithm**
- Finds **shortest path** from a source to all other vertices.
- Uses a **priority queue (min-heap)**.
- **Time Complexity:** O((V + E) log V)
- **Applications:** GPS navigation, network routing, AI pathfinding.

---

## 🧠 Pseudocode Example (BFS)

```cpp
BFS(Graph, Start):
  Initialize queue Q
  Mark Start as visited
  Enqueue(Start)
  while Q is not empty:
      Node = Q.dequeue()
      print(Node)
      for each neighbor of Node:
          if not visited:
              Mark as visited
              Enqueue(neighbor)
