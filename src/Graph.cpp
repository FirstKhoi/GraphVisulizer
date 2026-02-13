//
//  Graph.cpp
//  Graph Visualization - Data Structures & Algorithms Learning Tool
//
//  Created by Luong Nhat Khoi on 1/12/25.
//

#include "Graph.h"
#include <iostream>
#include <sstream>

const double INF = std::numeric_limits<double>::infinity();

Graph::Graph() : nextId(0), directed(false) {}

//  Basic Operations 

void Graph::addNode(float x, float y) {
    // Find the smallest available ID (reuse deleted IDs)
    std::set<int> usedIds;
    for (const auto& n : nodes) {
        usedIds.insert(n.id);
    }
    
    int newId = 0;
    while (usedIds.count(newId)) {
        newId++;
    }
    
    nodes.push_back({newId, x, y, -1});
    
    // Update nextId if needed
    if (newId >= nextId) {
        nextId = newId + 1;
    }
}

void Graph::addEdge(int u, int v, double weight, bool isDirected) {
    // Check if edge already exists
    for (const auto& e : edges) {
        if (e.source == u && e.target == v) return;
        if (!directed && !isDirected && e.source == v && e.target == u) return;
    }
    edges.push_back(Edge(u, v, weight, isDirected || directed));
}

void Graph::removeNode(int id) {
    // Remove all edges connected to this node
    edges.erase(std::remove_if(edges.begin(), edges.end(),
        [id](const Edge& e) { return e.source == id || e.target == id; }), edges.end());
    
    // Remove the node
    nodes.erase(std::remove_if(nodes.begin(), nodes.end(),
        [id](const Node& n) { return n.id == id; }), nodes.end());
}

void Graph::removeEdge(int u, int v) {
    edges.erase(std::remove_if(edges.begin(), edges.end(),
        [u, v, this](const Edge& e) {
            if (directed) return e.source == u && e.target == v;
            return (e.source == u && e.target == v) || (e.source == v && e.target == u);
        }), edges.end());
}

void Graph::clear() {
    nodes.clear();
    edges.clear();
    nextId = 0;
}

void Graph::setDirected(bool dir) {
    directed = dir;
    for (auto& e : edges) {
        e.directed = dir;
    }
}

bool Graph::isDirectedGraph() const {
    return directed;
}

void Graph::setEdgeWeight(int u, int v, double weight) {
    for (auto& e : edges) {
        if ((e.source == u && e.target == v) || 
            (!directed && e.source == v && e.target == u)) {
            e.weight = weight;
            e.capacity = weight;
            return;
        }
    }
}

//  Node/Edge Access 

int Graph::getNodeClicked(float x, float y, float radius) {
    for (const auto& n : nodes) {
        float dx = n.x - x;
        float dy = n.y - y;
        if (std::sqrt(dx*dx + dy*dy) <= radius) {
            return n.id;
        }
    }
    return -1;
}

const std::vector<Node>& Graph::getNodes() const { return nodes; }
const std::vector<Edge>& Graph::getEdges() const { return edges; }
std::vector<Node>& Graph::getNodesMutable() { return nodes; }
std::vector<Edge>& Graph::getEdgesMutable() { return edges; }

Node* Graph::getNodeById(int id) {
    for (auto& n : nodes) {
        if (n.id == id) return &n;
    }
    return nullptr;
}

Edge* Graph::getEdge(int u, int v) {
    for (auto& e : edges) {
        if (e.source == u && e.target == v) return &e;
        if (!directed && e.source == v && e.target == u) return &e;
    }
    return nullptr;
}

int Graph::getNodeCount() const { return nodes.size(); }
int Graph::getEdgeCount() const { return edges.size(); }

//  Graph Representations 

std::vector<std::vector<double>> Graph::getAdjacencyMatrix() const {
    if (nodes.empty()) return {};
    
    // Create ID to index mapping
    std::map<int, int> idToIdx;
    int idx = 0;
    for (const auto& n : nodes) {
        idToIdx[n.id] = idx++;
    }
    
    int n = nodes.size();
    std::vector<std::vector<double>> matrix(n, std::vector<double>(n, 0));
    
    for (const auto& e : edges) {
        int i = idToIdx[e.source];
        int j = idToIdx[e.target];
        matrix[i][j] = e.weight;
        if (!directed) {
            matrix[j][i] = e.weight;
        }
    }
    
    return matrix;
}

std::map<int, std::vector<std::pair<int, double>>> Graph::getAdjacencyList() const {
    std::map<int, std::vector<std::pair<int, double>>> adjList;
    
    // Initialize all nodes
    for (const auto& n : nodes) {
        adjList[n.id] = {};
    }
    
    for (const auto& e : edges) {
        adjList[e.source].push_back({e.target, e.weight});
        if (!directed) {
            adjList[e.target].push_back({e.source, e.weight});
        }
    }
    
    return adjList;
}

std::vector<std::tuple<int, int, double>> Graph::getEdgeList() const {
    std::vector<std::tuple<int, int, double>> edgeList;
    for (const auto& e : edges) {
        edgeList.push_back({e.source, e.target, e.weight});
    }
    return edgeList;
}

void Graph::loadFromAdjacencyMatrix(const std::vector<std::vector<double>>& matrix, bool isDir) {
    clear();
    directed = isDir;
    
    int n = matrix.size();
    
    // Create nodes in a circle layout
    float centerX = 600, centerY = 400;
    float radius = 200;
    for (int i = 0; i < n; i++) {
        float angle = 2 * M_PI * i / n - M_PI / 2;
        float x = centerX + radius * cos(angle);
        float y = centerY + radius * sin(angle);
        nodes.push_back({i, x, y, -1});
    }
    nextId = n;
    
    // Create edges
    for (int i = 0; i < n; i++) {
        for (int j = directed ? 0 : i; j < n; j++) {
            if (matrix[i][j] != 0 && i != j) {
                edges.push_back(Edge(i, j, matrix[i][j], directed));
            }
        }
    }
}

void Graph::loadFromAdjacencyList(const std::map<int, std::vector<std::pair<int, double>>>& adjList, bool isDir) {
    clear();
    directed = isDir;
    
    // Create nodes
    std::set<int> nodeIds;
    for (const auto& [id, neighbors] : adjList) {
        nodeIds.insert(id);
        for (const auto& [neighbor, w] : neighbors) {
            nodeIds.insert(neighbor);
        }
    }
    
    int n = nodeIds.size();
    float centerX = 600, centerY = 400;
    float radius = 200;
    int idx = 0;
    std::map<int, int> idMap;
    
    for (int id : nodeIds) {
        float angle = 2 * M_PI * idx / n - M_PI / 2;
        float x = centerX + radius * cos(angle);
        float y = centerY + radius * sin(angle);
        nodes.push_back({id, x, y, -1});
        idMap[id] = idx++;
        if (id >= nextId) nextId = id + 1;
    }
    
    // Create edges
    std::set<std::pair<int,int>> addedEdges;
    for (const auto& [u, neighbors] : adjList) {
        for (const auto& [v, w] : neighbors) {
            if (directed) {
                edges.push_back(Edge(u, v, w, true));
            } else {
                auto edge = std::make_pair(std::min(u,v), std::max(u,v));
                if (addedEdges.find(edge) == addedEdges.end()) {
                    edges.push_back(Edge(u, v, w, false));
                    addedEdges.insert(edge);
                }
            }
        }
    }
}

void Graph::loadFromEdgeList(const std::vector<std::tuple<int, int, double>>& edgeList, bool isDir) {
    clear();
    directed = isDir;
    
    // Find all unique nodes
    std::set<int> nodeIds;
    for (const auto& [u, v, w] : edgeList) {
        nodeIds.insert(u);
        nodeIds.insert(v);
    }
    
    int n = nodeIds.size();
    float centerX = 600, centerY = 400;
    float radius = 200;
    int idx = 0;
    
    for (int id : nodeIds) {
        float angle = 2 * M_PI * idx / n - M_PI / 2;
        float x = centerX + radius * cos(angle);
        float y = centerY + radius * sin(angle);
        nodes.push_back({id, x, y, -1});
        if (id >= nextId) nextId = id + 1;
        idx++;
    }
    
    // Create edges
    for (const auto& [u, v, w] : edgeList) {
        edges.push_back(Edge(u, v, w, directed));
    }
}

//  Utility Functions 

std::vector<int> Graph::getNeighbors(int nodeId) const {
    std::vector<int> neighbors;
    for (const auto& e : edges) {
        if (e.source == nodeId) {
            neighbors.push_back(e.target);
        }
        if (!directed && e.target == nodeId) {
            neighbors.push_back(e.source);
        }
    }
    return neighbors;
}

int Graph::getDegree(int nodeId) const {
    int degree = 0;
    for (const auto& e : edges) {
        if (e.source == nodeId || e.target == nodeId) {
            if (!directed) degree++;
            else if (e.source == nodeId) degree++;
        }
    }
    return degree;
}

int Graph::getInDegree(int nodeId) const {
    int degree = 0;
    for (const auto& e : edges) {
        if (e.target == nodeId) degree++;
    }
    return degree;
}

int Graph::getOutDegree(int nodeId) const {
    int degree = 0;
    for (const auto& e : edges) {
        if (e.source == nodeId) degree++;
    }
    return degree;
}

bool Graph::isConnected() {
    if (nodes.empty()) return true;
    
    std::set<int> visited;
    std::queue<int> q;
    q.push(nodes[0].id);
    visited.insert(nodes[0].id);
    
    while (!q.empty()) {
        int curr = q.front();
        q.pop();
        
        for (int neighbor : getNeighbors(curr)) {
            if (visited.find(neighbor) == visited.end()) {
                visited.insert(neighbor);
                q.push(neighbor);
            }
        }
        
        // For directed graphs, also check reverse edges
        if (directed) {
            for (const auto& e : edges) {
                if (e.target == curr && visited.find(e.source) == visited.end()) {
                    visited.insert(e.source);
                    q.push(e.source);
                }
            }
        }
    }
    
    return visited.size() == nodes.size();
}

//  BFS Algorithm 

std::vector<AlgorithmStep> Graph::bfs(int startNode) {
    std::vector<AlgorithmStep> steps;
    
    if (getNodeById(startNode) == nullptr) {
        AlgorithmStep errorStep;
        errorStep.description = "Error: Start node not found!";
        steps.push_back(errorStep);
        return steps;
    }
    
    std::set<int> visited;
    std::queue<int> q;
    
    // Initial step
    AlgorithmStep initStep;
    initStep.highlightedNodes = {startNode};
    initStep.description = "BFS: Starting from node " + std::to_string(startNode);
    steps.push_back(initStep);
    
    q.push(startNode);
    visited.insert(startNode);
    
    std::vector<int> visitOrder;
    visitOrder.push_back(startNode);
    
    while (!q.empty()) {
        int curr = q.front();
        q.pop();
        
        AlgorithmStep step;
        step.visitedNodes = visitOrder;
        step.highlightedNodes = {curr};
        step.description = "BFS: Visiting node " + std::to_string(curr);
        
        std::vector<int> neighbors = getNeighbors(curr);
        std::sort(neighbors.begin(), neighbors.end());
        
        for (int neighbor : neighbors) {
            if (visited.find(neighbor) == visited.end()) {
                visited.insert(neighbor);
                q.push(neighbor);
                visitOrder.push_back(neighbor);
                
                step.highlightedEdges.push_back({curr, neighbor});
                step.description += ", adding neighbor " + std::to_string(neighbor);
            }
        }
        
        steps.push_back(step);
    }
    
    // Final step
    AlgorithmStep finalStep;
    finalStep.visitedNodes = visitOrder;
    finalStep.description = "BFS Complete! Visit order: ";
    for (size_t i = 0; i < visitOrder.size(); i++) {
        finalStep.description += std::to_string(visitOrder[i]);
        if (i < visitOrder.size() - 1) finalStep.description += " -> ";
    }
    steps.push_back(finalStep);
    
    return steps;
}

//  DFS Algorithm 

std::vector<AlgorithmStep> Graph::dfs(int startNode) {
    std::vector<AlgorithmStep> steps;
    
    if (getNodeById(startNode) == nullptr) {
        AlgorithmStep errorStep;
        errorStep.description = "Error: Start node not found!";
        steps.push_back(errorStep);
        return steps;
    }
    
    std::set<int> visited;
    std::stack<int> s;
    
    // Initial step
    AlgorithmStep initStep;
    initStep.highlightedNodes = {startNode};
    initStep.description = "DFS: Starting from node " + std::to_string(startNode);
    steps.push_back(initStep);
    
    s.push(startNode);
    std::vector<int> visitOrder;
    
    while (!s.empty()) {
        int curr = s.top();
        s.pop();
        
        if (visited.find(curr) != visited.end()) continue;
        
        visited.insert(curr);
        visitOrder.push_back(curr);
        
        AlgorithmStep step;
        step.visitedNodes = visitOrder;
        step.highlightedNodes = {curr};
        step.description = "DFS: Visiting node " + std::to_string(curr);
        
        std::vector<int> neighbors = getNeighbors(curr);
        std::sort(neighbors.rbegin(), neighbors.rend()); // Reverse to visit smaller first
        
        for (int neighbor : neighbors) {
            if (visited.find(neighbor) == visited.end()) {
                s.push(neighbor);
                step.highlightedEdges.push_back({curr, neighbor});
            }
        }
        
        steps.push_back(step);
    }
    
    // Final step
    AlgorithmStep finalStep;
    finalStep.visitedNodes = visitOrder;
    finalStep.description = "DFS Complete! Visit order: ";
    for (size_t i = 0; i < visitOrder.size(); i++) {
        finalStep.description += std::to_string(visitOrder[i]);
        if (i < visitOrder.size() - 1) finalStep.description += " -> ";
    }
    steps.push_back(finalStep);
    
    return steps;
}

//  Dijkstra's Algorithm 

std::vector<AlgorithmStep> Graph::dijkstra(int startNode, int endNode) {
    std::vector<AlgorithmStep> steps;
    
    if (getNodeById(startNode) == nullptr || getNodeById(endNode) == nullptr) {
        AlgorithmStep errorStep;
        errorStep.description = "Error: Start or end node not found!";
        steps.push_back(errorStep);
        return steps;
    }
    
    std::map<int, double> dist;
    std::map<int, int> prev;
    std::set<int> visited;
    
    // Priority queue: (distance, node)
    std::priority_queue<std::pair<double, int>, 
                       std::vector<std::pair<double, int>>,
                       std::greater<std::pair<double, int>>> pq;
    
    // Initialize distances
    for (const auto& n : nodes) {
        dist[n.id] = INF;
    }
    dist[startNode] = 0;
    pq.push({0, startNode});
    
    // Initial step
    AlgorithmStep initStep;
    initStep.highlightedNodes = {startNode};
    initStep.nodeValues = dist;
    initStep.description = "Dijkstra: Starting from node " + std::to_string(startNode) + 
                          " to node " + std::to_string(endNode);
    steps.push_back(initStep);
    
    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        
        if (visited.find(u) != visited.end()) continue;
        visited.insert(u);
        
        AlgorithmStep step;
        step.visitedNodes = std::vector<int>(visited.begin(), visited.end());
        step.highlightedNodes = {u};
        step.nodeValues = dist;
        step.description = "Dijkstra: Processing node " + std::to_string(u) + 
                          " (distance = " + std::to_string((int)d) + ")";
        
        if (u == endNode) {
            step.description += " - Found destination!";
            steps.push_back(step);
            break;
        }
        
        for (const auto& e : edges) {
            int v = -1;
            double w = 0;
            
            if (e.source == u) {
                v = e.target;
                w = e.weight;
            } else if (!directed && e.target == u) {
                v = e.source;
                w = e.weight;
            }
            
            if (v != -1 && visited.find(v) == visited.end()) {
                double newDist = dist[u] + w;
                if (newDist < dist[v]) {
                    dist[v] = newDist;
                    prev[v] = u;
                    pq.push({newDist, v});
                    step.highlightedEdges.push_back({u, v});
                }
            }
        }
        
        step.nodeValues = dist;
        steps.push_back(step);
    }
    
    // Reconstruct path
    std::vector<int> path;
    if (dist[endNode] != INF) {
        int curr = endNode;
        while (curr != startNode) {
            path.push_back(curr);
            curr = prev[curr];
        }
        path.push_back(startNode);
        std::reverse(path.begin(), path.end());
    }
    
    // Final step
    AlgorithmStep finalStep;
    finalStep.visitedNodes = std::vector<int>(visited.begin(), visited.end());
    finalStep.path = path;
    finalStep.nodeValues = dist;
    
    if (path.empty()) {
        finalStep.description = "Dijkstra Complete! No path found from " + 
                               std::to_string(startNode) + " to " + std::to_string(endNode);
    } else {
        finalStep.description = "Dijkstra Complete! Shortest path (distance = " + 
                               std::to_string((int)dist[endNode]) + "): ";
        for (size_t i = 0; i < path.size(); i++) {
            finalStep.description += std::to_string(path[i]);
            if (i < path.size() - 1) finalStep.description += " -> ";
            if (i < path.size() - 1) {
                finalStep.highlightedEdges.push_back({path[i], path[i+1]});
            }
        }
    }
    steps.push_back(finalStep);
    
    return steps;
}

//  Bipartite Check 

std::pair<bool, std::vector<int>> Graph::checkBipartite() {
    std::vector<int> coloring(nextId, -1);
    
    if (nodes.empty()) return {true, coloring};
    
    bool isBipartite = true;
    
    for (const auto& startNode : nodes) {
        if (coloring[startNode.id] != -1) continue;
        
        std::queue<int> q;
        q.push(startNode.id);
        coloring[startNode.id] = 0;
        
        while (!q.empty() && isBipartite) {
            int u = q.front();
            q.pop();
            
            for (int v : getNeighbors(u)) {
                if (coloring[v] == -1) {
                    coloring[v] = 1 - coloring[u];
                    q.push(v);
                } else if (coloring[v] == coloring[u]) {
                    isBipartite = false;
                    break;
                }
            }
        }
    }
    
    // Update node colors in the graph
    for (auto& n : nodes) {
        n.color = coloring[n.id];
    }
    
    return {isBipartite, coloring};
}

//  Prim's MST Algorithm 

std::vector<AlgorithmStep> Graph::primMST(int startNode) {
    std::vector<AlgorithmStep> steps;
    
    if (getNodeById(startNode) == nullptr) {
        AlgorithmStep errorStep;
        errorStep.description = "Error: Start node not found!";
        steps.push_back(errorStep);
        return steps;
    }
    
    if (!isConnected()) {
        AlgorithmStep errorStep;
        errorStep.description = "Error: Graph is not connected! MST cannot be formed.";
        steps.push_back(errorStep);
        return steps;
    }
    
    std::set<int> inMST;
    std::vector<std::pair<int,int>> mstEdges;
    double totalWeight = 0;
    
    // Priority queue: (weight, from, to)
    std::priority_queue<std::tuple<double, int, int>,
                       std::vector<std::tuple<double, int, int>>,
                       std::greater<std::tuple<double, int, int>>> pq;
    
    inMST.insert(startNode);
    
    // Initial step
    AlgorithmStep initStep;
    initStep.highlightedNodes = {startNode};
    initStep.description = "Prim's MST: Starting from node " + std::to_string(startNode);
    steps.push_back(initStep);
    
    // Add all edges from start node
    for (const auto& e : edges) {
        if (e.source == startNode) {
            pq.push({e.weight, startNode, e.target});
        }
        if (!directed && e.target == startNode) {
            pq.push({e.weight, startNode, e.source});
        }
    }
    
    while (!pq.empty() && inMST.size() < nodes.size()) {
        auto [w, u, v] = pq.top();
        pq.pop();
        
        if (inMST.find(v) != inMST.end()) continue;
        
        inMST.insert(v);
        mstEdges.push_back({u, v});
        totalWeight += w;
        
        AlgorithmStep step;
        step.visitedNodes = std::vector<int>(inMST.begin(), inMST.end());
        step.highlightedNodes = {v};
        step.highlightedEdges = {{u, v}};
        step.visitedEdges = mstEdges;
        step.description = "Prim: Adding edge (" + std::to_string(u) + ", " + 
                          std::to_string(v) + ") with weight " + std::to_string((int)w) +
                          ". Total weight: " + std::to_string((int)totalWeight);
        steps.push_back(step);
        
        // Add edges from new node
        for (const auto& e : edges) {
            if (e.source == v && inMST.find(e.target) == inMST.end()) {
                pq.push({e.weight, v, e.target});
            }
            if (!directed && e.target == v && inMST.find(e.source) == inMST.end()) {
                pq.push({e.weight, v, e.source});
            }
        }
    }
    
    // Final step
    AlgorithmStep finalStep;
    finalStep.visitedNodes = std::vector<int>(inMST.begin(), inMST.end());
    finalStep.visitedEdges = mstEdges;
    finalStep.description = "Prim's MST Complete! Total weight: " + std::to_string((int)totalWeight) +
                           ", Edges: " + std::to_string(mstEdges.size());
    steps.push_back(finalStep);
    
    return steps;
}

//  Kruskal's MST Algorithm 

int Graph::findParent(int node, std::map<int, int>& parent) {
    if (parent[node] != node) {
        parent[node] = findParent(parent[node], parent);
    }
    return parent[node];
}

void Graph::unionSets(int a, int b, std::map<int, int>& parent, std::map<int, int>& rank) {
    int rootA = findParent(a, parent);
    int rootB = findParent(b, parent);
    
    if (rootA != rootB) {
        if (rank[rootA] < rank[rootB]) std::swap(rootA, rootB);
        parent[rootB] = rootA;
        if (rank[rootA] == rank[rootB]) rank[rootA]++;
    }
}

std::vector<AlgorithmStep> Graph::kruskalMST() {
    std::vector<AlgorithmStep> steps;
    
    if (!isConnected()) {
        AlgorithmStep errorStep;
        errorStep.description = "Error: Graph is not connected! MST cannot be formed.";
        steps.push_back(errorStep);
        return steps;
    }
    
    // Sort edges by weight
    std::vector<Edge> sortedEdges = edges;
    std::sort(sortedEdges.begin(), sortedEdges.end(),
        [](const Edge& a, const Edge& b) { return a.weight < b.weight; });
    
    // Initialize Union-Find
    std::map<int, int> parent, rank;
    for (const auto& n : nodes) {
        parent[n.id] = n.id;
        rank[n.id] = 0;
    }
    
    std::vector<std::pair<int,int>> mstEdges;
    double totalWeight = 0;
    
    // Initial step
    AlgorithmStep initStep;
    initStep.description = "Kruskal's MST: Sorting edges by weight...";
    steps.push_back(initStep);
    
    for (const auto& e : sortedEdges) {
        if (mstEdges.size() >= nodes.size() - 1) break;
        
        int rootU = findParent(e.source, parent);
        int rootV = findParent(e.target, parent);
        
        AlgorithmStep step;
        step.highlightedEdges = {{e.source, e.target}};
        step.visitedEdges = mstEdges;
        
        if (rootU != rootV) {
            unionSets(e.source, e.target, parent, rank);
            mstEdges.push_back({e.source, e.target});
            totalWeight += e.weight;
            
            step.highlightedNodes = {e.source, e.target};
            step.description = "Kruskal: Adding edge (" + std::to_string(e.source) + ", " + 
                              std::to_string(e.target) + ") with weight " + std::to_string((int)e.weight) +
                              ". Total weight: " + std::to_string((int)totalWeight);
        } else {
            step.description = "Kruskal: Skipping edge (" + std::to_string(e.source) + ", " + 
                              std::to_string(e.target) + ") - would create cycle";
        }
        
        steps.push_back(step);
    }
    
    // Final step
    AlgorithmStep finalStep;
    for (const auto& n : nodes) finalStep.visitedNodes.push_back(n.id);
    finalStep.visitedEdges = mstEdges;
    finalStep.description = "Kruskal's MST Complete! Total weight: " + std::to_string((int)totalWeight) +
                           ", Edges: " + std::to_string(mstEdges.size());
    steps.push_back(finalStep);
    
    return steps;
}

//  Ford-Fulkerson Algorithm 

bool Graph::bfsFF(int source, int sink, std::map<int, int>& parent, 
                  std::map<std::pair<int,int>, double>& residual) {
    std::set<int> visited;
    std::queue<int> q;
    
    q.push(source);
    visited.insert(source);
    parent[source] = -1;
    
    while (!q.empty()) {
        int u = q.front();
        q.pop();
        
        for (const auto& n : nodes) {
            int v = n.id;
            if (visited.find(v) == visited.end() && residual[{u, v}] > 0) {
                parent[v] = u;
                if (v == sink) return true;
                visited.insert(v);
                q.push(v);
            }
        }
    }
    
    return false;
}

std::vector<AlgorithmStep> Graph::fordFulkerson(int source, int sink) {
    std::vector<AlgorithmStep> steps;
    
    if (getNodeById(source) == nullptr || getNodeById(sink) == nullptr) {
        AlgorithmStep errorStep;
        errorStep.description = "Error: Source or sink node not found!";
        steps.push_back(errorStep);
        return steps;
    }
    
    if (source == sink) {
        AlgorithmStep errorStep;
        errorStep.description = "Error: Source and sink must be different!";
        steps.push_back(errorStep);
        return steps;
    }
    
    // Initialize residual capacities
    std::map<std::pair<int,int>, double> residual;
    for (const auto& n1 : nodes) {
        for (const auto& n2 : nodes) {
            residual[{n1.id, n2.id}] = 0;
        }
    }
    
    for (const auto& e : edges) {
        residual[{e.source, e.target}] = e.weight;
        if (!directed) {
            residual[{e.target, e.source}] = e.weight;
        }
    }
    
    double maxFlow = 0;
    std::map<int, int> parent;
    
    // Initial step
    AlgorithmStep initStep;
    initStep.highlightedNodes = {source, sink};
    initStep.description = "Ford-Fulkerson: Finding max flow from " + std::to_string(source) + 
                          " to " + std::to_string(sink);
    steps.push_back(initStep);
    
    int iteration = 0;
    while (bfsFF(source, sink, parent, residual)) {
        iteration++;
        
        // Find minimum residual capacity along the path
        double pathFlow = INF;
        std::vector<int> path;
        for (int v = sink; v != source; v = parent[v]) {
            int u = parent[v];
            pathFlow = std::min(pathFlow, residual[{u, v}]);
            path.push_back(v);
        }
        path.push_back(source);
        std::reverse(path.begin(), path.end());
        
        // Update residual capacities
        AlgorithmStep step;
        step.path = path;
        step.highlightedNodes = {source, sink};
        step.description = "Iteration " + std::to_string(iteration) + ": Found path with flow " + 
                          std::to_string((int)pathFlow) + ": ";
        
        for (int v = sink; v != source; v = parent[v]) {
            int u = parent[v];
            residual[{u, v}] -= pathFlow;
            residual[{v, u}] += pathFlow;
            step.highlightedEdges.push_back({u, v});
        }
        
        for (size_t i = 0; i < path.size(); i++) {
            step.description += std::to_string(path[i]);
            if (i < path.size() - 1) step.description += " -> ";
        }
        
        maxFlow += pathFlow;
        step.description += ". Current max flow: " + std::to_string((int)maxFlow);
        steps.push_back(step);
        
        parent.clear();
    }
    
    // Final step
    AlgorithmStep finalStep;
    finalStep.highlightedNodes = {source, sink};
    finalStep.description = "Ford-Fulkerson Complete! Maximum Flow: " + std::to_string((int)maxFlow);
    steps.push_back(finalStep);
    
    return steps;
}

//  Eulerian Path/Circuit 

std::pair<bool, bool> Graph::hasEulerianPathOrCircuit() {
    if (nodes.empty()) return {false, false};
    if (!isConnected()) return {false, false};
    
    int oddDegreeCount = 0;
    
    if (directed) {
        // For directed graph: check in-degree == out-degree for all nodes
        std::map<int, int> inDeg, outDeg;
        for (const auto& n : nodes) {
            inDeg[n.id] = 0;
            outDeg[n.id] = 0;
        }
        for (const auto& e : edges) {
            outDeg[e.source]++;
            inDeg[e.target]++;
        }
        
        int startNodes = 0, endNodes = 0;
        for (const auto& n : nodes) {
            int diff = outDeg[n.id] - inDeg[n.id];
            if (diff == 1) startNodes++;
            else if (diff == -1) endNodes++;
            else if (diff != 0) return {false, false};
        }
        
        if (startNodes == 0 && endNodes == 0) return {true, true};  // Circuit
        if (startNodes == 1 && endNodes == 1) return {true, false}; // Path
        return {false, false};
    } else {
        // For undirected graph: count odd degree vertices
        for (const auto& n : nodes) {
            if (getDegree(n.id) % 2 == 1) oddDegreeCount++;
        }
        
        if (oddDegreeCount == 0) return {true, true};   // Circuit
        if (oddDegreeCount == 2) return {true, false};  // Path
        return {false, false};
    }
}

int Graph::dfsCount(int node, std::set<int>& visited, std::set<std::pair<int,int>>& removedEdges) {
    visited.insert(node);
    int count = 1;
    
    for (const auto& e : edges) {
        int neighbor = -1;
        std::pair<int,int> edgePair;
        
        if (e.source == node) {
            neighbor = e.target;
            edgePair = {e.source, e.target};
        } else if (!directed && e.target == node) {
            neighbor = e.source;
            edgePair = {e.target, e.source};
        }
        
        if (neighbor != -1 && visited.find(neighbor) == visited.end()) {
            auto reverseEdge = std::make_pair(edgePair.second, edgePair.first);
            if (removedEdges.find(edgePair) == removedEdges.end() &&
                removedEdges.find(reverseEdge) == removedEdges.end()) {
                count += dfsCount(neighbor, visited, removedEdges);
            }
        }
    }
    
    return count;
}

bool Graph::isValidNextEdge(int u, int v, std::set<std::pair<int,int>>& removedEdges) {
    // Count adjacent vertices
    int count = 0;
    for (const auto& e : edges) {
        std::pair<int,int> edgePair;
        if (e.source == u) edgePair = {e.source, e.target};
        else if (!directed && e.target == u) edgePair = {e.target, e.source};
        else continue;
        
        auto reverseEdge = std::make_pair(edgePair.second, edgePair.first);
        if (removedEdges.find(edgePair) == removedEdges.end() &&
            removedEdges.find(reverseEdge) == removedEdges.end()) {
            count++;
        }
    }
    
    if (count == 1) return true;  // Only one edge, must use it
    
    // Check if removing edge (u,v) disconnects the graph
    std::set<int> visited1;
    int count1 = dfsCount(u, visited1, removedEdges);
    
    removedEdges.insert({u, v});
    std::set<int> visited2;
    int count2 = dfsCount(u, visited2, removedEdges);
    removedEdges.erase({u, v});
    
    return count1 <= count2;
}

std::vector<AlgorithmStep> Graph::fleuryEulerian(int startNode) {
    std::vector<AlgorithmStep> steps;
    
    auto [hasPath, hasCircuit] = hasEulerianPathOrCircuit();
    if (!hasPath) {
        AlgorithmStep errorStep;
        errorStep.description = "Error: Graph does not have an Eulerian path or circuit!";
        steps.push_back(errorStep);
        return steps;
    }
    
    // Initial step
    AlgorithmStep initStep;
    initStep.highlightedNodes = {startNode};
    initStep.description = "Fleury's Algorithm: Finding Eulerian " + 
                          std::string(hasCircuit ? "Circuit" : "Path") + 
                          " starting from node " + std::to_string(startNode);
    steps.push_back(initStep);
    
    std::set<std::pair<int,int>> removedEdges;
    std::vector<int> path;
    path.push_back(startNode);
    
    int current = startNode;
    
    while (true) {
        std::vector<int> neighbors;
        for (const auto& e : edges) {
            int neighbor = -1;
            std::pair<int,int> edgePair;
            
            if (e.source == current) {
                neighbor = e.target;
                edgePair = {e.source, e.target};
            } else if (!directed && e.target == current) {
                neighbor = e.source;
                edgePair = {e.target, e.source};
            }
            
            if (neighbor != -1) {
                auto reverseEdge = std::make_pair(edgePair.second, edgePair.first);
                if (removedEdges.find(edgePair) == removedEdges.end() &&
                    removedEdges.find(reverseEdge) == removedEdges.end()) {
                    neighbors.push_back(neighbor);
                }
            }
        }
        
        if (neighbors.empty()) break;
        
        int next = -1;
        for (int n : neighbors) {
            if (isValidNextEdge(current, n, removedEdges)) {
                next = n;
                break;
            }
        }
        
        if (next == -1) next = neighbors[0];
        
        removedEdges.insert({current, next});
        removedEdges.insert({next, current});
        path.push_back(next);
        
        AlgorithmStep step;
        step.path = path;
        step.highlightedEdges = {{current, next}};
        step.visitedNodes = path;
        step.description = "Fleury: Moving from " + std::to_string(current) + 
                          " to " + std::to_string(next);
        steps.push_back(step);
        
        current = next;
    }
    
    // Final step
    AlgorithmStep finalStep;
    finalStep.path = path;
    finalStep.visitedNodes = path;
    finalStep.description = "Fleury Complete! Eulerian " + 
                           std::string(hasCircuit ? "Circuit" : "Path") + ": ";
    for (size_t i = 0; i < path.size(); i++) {
        finalStep.description += std::to_string(path[i]);
        if (i < path.size() - 1) finalStep.description += " -> ";
    }
    steps.push_back(finalStep);
    
    return steps;
}

std::vector<AlgorithmStep> Graph::hierholzerEulerian(int startNode) {
    std::vector<AlgorithmStep> steps;
    
    auto [hasPath, hasCircuit] = hasEulerianPathOrCircuit();
    if (!hasPath) {
        AlgorithmStep errorStep;
        errorStep.description = "Error: Graph does not have an Eulerian path or circuit!";
        steps.push_back(errorStep);
        return steps;
    }
    
    // Initial step
    AlgorithmStep initStep;
    initStep.highlightedNodes = {startNode};
    initStep.description = "Hierholzer's Algorithm: Finding Eulerian " + 
                          std::string(hasCircuit ? "Circuit" : "Path") + 
                          " starting from node " + std::to_string(startNode);
    steps.push_back(initStep);
    
    // Build adjacency list with edge count
    std::map<int, std::vector<int>> adj;
    for (const auto& e : edges) {
        adj[e.source].push_back(e.target);
        if (!directed) {
            adj[e.target].push_back(e.source);
        }
    }
    
    std::stack<int> currPath;
    std::vector<int> circuit;
    
    currPath.push(startNode);
    int curr = startNode;
    
    while (!currPath.empty()) {
        if (!adj[curr].empty()) {
            currPath.push(curr);
            int next = adj[curr].back();
            adj[curr].pop_back();
            
            // Remove reverse edge for undirected graph
            if (!directed) {
                auto it = std::find(adj[next].begin(), adj[next].end(), curr);
                if (it != adj[next].end()) {
                    adj[next].erase(it);
                }
            }
            
            AlgorithmStep step;
            step.highlightedEdges = {{curr, next}};
            step.highlightedNodes = {next};
            step.description = "Hierholzer: Moving from " + std::to_string(curr) + 
                              " to " + std::to_string(next);
            steps.push_back(step);
            
            curr = next;
        } else {
            circuit.push_back(curr);
            curr = currPath.top();
            currPath.pop();
            
            AlgorithmStep step;
            step.highlightedNodes = {curr};
            step.path = circuit;
            step.description = "Hierholzer: Backtracking, adding " + std::to_string(circuit.back()) + 
                              " to circuit";
            steps.push_back(step);
        }
    }
    
    std::reverse(circuit.begin(), circuit.end());
    
    // Final step
    AlgorithmStep finalStep;
    finalStep.path = circuit;
    finalStep.visitedNodes = circuit;
    finalStep.description = "Hierholzer Complete! Eulerian " + 
                           std::string(hasCircuit ? "Circuit" : "Path") + ": ";
    for (size_t i = 0; i < circuit.size(); i++) {
        finalStep.description += std::to_string(circuit[i]);
        if (i < circuit.size() - 1) finalStep.description += " -> ";
    }
    steps.push_back(finalStep);
    
    return steps;
}
