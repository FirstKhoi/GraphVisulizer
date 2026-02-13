//
//  Graph.h
//  Graph Visualization - Data Structures & Algorithms Learning Tool
//
//  Created by Luong Nhat Khoi on 1/12/25.
//

#pragma once
#include <vector>
#include <string>
#include <cmath>
#include <map>
#include <set>
#include <queue>
#include <stack>
#include <limits>
#include <algorithm>
#include <functional>
#include <tuple>

struct Node {
    int id;
    float x, y;
    int color = -1;
};

struct Edge {
    int source;
    int target;
    double weight;
    bool directed;
    double flow = 0;
    double capacity = 0;
    
    Edge() : source(0), target(0), weight(1.0), directed(false), flow(0), capacity(1.0) {}
    
    Edge(int s, int t, double w = 1.0, bool dir = false) 
        : source(s), target(t), weight(w), directed(dir), flow(0), capacity(w) {}
};


struct AlgorithmStep {
    std::vector<int> highlightedNodes;
    std::vector<std::pair<int,int>> highlightedEdges;
    std::vector<int> visitedNodes;
    std::vector<std::pair<int,int>> visitedEdges;
    std::string description;
    std::map<int, double> nodeValues;
    std::map<std::pair<int,int>, double> edgeValues;
    std::vector<int> path;
};

class Graph {
public:
    Graph();
    
    void addNode(float x, float y);
    void addEdge(int u, int v, double weight = 1.0, bool directed = false);
    void removeNode(int id);
    void removeEdge(int u, int v);
    void clear();
    void setDirected(bool dir);
    bool isDirectedGraph() const;
    void setEdgeWeight(int u, int v, double weight);

    int getNodeClicked(float x, float y, float radius = 25.0f);
    const std::vector<Node>& getNodes() const;
    const std::vector<Edge>& getEdges() const;
    std::vector<Node>& getNodesMutable();
    std::vector<Edge>& getEdgesMutable();
    Node* getNodeById(int id);
    Edge* getEdge(int u, int v);
    int getNodeCount() const;
    int getEdgeCount() const;
    
    std::vector<std::vector<double>> getAdjacencyMatrix() const;
    std::map<int, std::vector<std::pair<int, double>>> getAdjacencyList() const;
    std::vector<std::tuple<int, int, double>> getEdgeList() const;
    void loadFromAdjacencyMatrix(const std::vector<std::vector<double>>& matrix, bool isDir = false);
    void loadFromAdjacencyList(const std::map<int, std::vector<std::pair<int, double>>>& adjList, bool isDir = false);
    void loadFromEdgeList(const std::vector<std::tuple<int, int, double>>& edgeList, bool isDir = false);
    
    std::vector<AlgorithmStep> bfs(int startNode);
    std::vector<AlgorithmStep> dfs(int startNode);
    std::vector<AlgorithmStep> dijkstra(int startNode, int endNode);
    std::pair<bool, std::vector<int>> checkBipartite();
    
    std::vector<AlgorithmStep> primMST(int startNode);
    std::vector<AlgorithmStep> kruskalMST();
    
    std::vector<AlgorithmStep> fordFulkerson(int source, int sink);

    std::vector<AlgorithmStep> fleuryEulerian(int startNode);
    std::vector<AlgorithmStep> hierholzerEulerian(int startNode);
    std::pair<bool, bool> hasEulerianPathOrCircuit();
    
    bool isConnected();
    std::vector<int> getNeighbors(int nodeId) const;
    int getDegree(int nodeId) const;
    int getInDegree(int nodeId) const;
    int getOutDegree(int nodeId) const;

private:
    std::vector<Node> nodes;
    std::vector<Edge> edges;
    int nextId = 0;
    bool directed = false;
    
    bool bfsFF(int source, int sink, std::map<int, int>& parent, 
               std::map<std::pair<int,int>, double>& residual);
    
    bool isValidNextEdge(int u, int v, std::set<std::pair<int,int>>& removedEdges);
    int dfsCount(int node, std::set<int>& visited, std::set<std::pair<int,int>>& removedEdges);
    
    int findParent(int node, std::map<int, int>& parent);
    void unionSets(int a, int b, std::map<int, int>& parent, std::map<int, int>& rank);
};
