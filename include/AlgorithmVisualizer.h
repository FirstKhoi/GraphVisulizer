//
//  AlgorithmVisualizer.h
//  Graph Visualization - Data Structures & Algorithms Learning Tool
//
//  Created by Luong Nhat Khoi on 1/12/25.
//

#pragma once
#include "Graph.h"
#include <vector>
#include <string>
#include <chrono>

enum class AlgorithmType {
    NONE,
    BFS,
    DFS,
    DIJKSTRA,
    BIPARTITE,
    PRIM_MST,
    KRUSKAL_MST,
    FORD_FULKERSON,
    FLEURY_EULERIAN,
    HIERHOLZER_EULERIAN
};

class AlgorithmVisualizer {
public:
    AlgorithmVisualizer();

    void runBFS(Graph& graph, int startNode);
    void runDFS(Graph& graph, int startNode);
    void runDijkstra(Graph& graph, int startNode, int endNode);
    void runBipartiteCheck(Graph& graph);
    void runPrimMST(Graph& graph, int startNode);
    void runKruskalMST(Graph& graph);
    void runFordFulkerson(Graph& graph, int source, int sink);
    void runFleuryEulerian(Graph& graph, int startNode);
    void runHierholzerEulerian(Graph& graph, int startNode);

    void nextStep();
    void prevStep();
    void reset();
    void setAutoPlay(bool autoPlay, float speed = 1.0f);
    void update();

    bool isRunning() const;
    bool isFinished() const;
    int getCurrentStepIndex() const;
    int getTotalSteps() const;
    const AlgorithmStep& getCurrentStep() const;
    AlgorithmType getCurrentAlgorithm() const;
    std::string getCurrentDescription() const;
    
    const std::vector<int>& getHighlightedNodes() const;
    const std::vector<std::pair<int,int>>& getHighlightedEdges() const;
    const std::vector<int>& getVisitedNodes() const;
    const std::vector<std::pair<int,int>>& getVisitedEdges() const;
    const std::vector<int>& getPath() const;
    const std::map<int, double>& getNodeValues() const;
    
    void clear();

private:
    std::vector<AlgorithmStep> steps;
    int currentStep;
    AlgorithmType currentAlgorithm;
    bool running;
    bool autoPlay;
    float playSpeed;
    std::chrono::steady_clock::time_point lastStepTime;
    
    bool bipartiteResult;
    std::vector<int> bipartiteColoring;
    
    AlgorithmStep emptyStep;
};
