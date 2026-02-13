//
//  AlgorithmVisualizer.cpp
//  Graph Visualization - Data Structures & Algorithms Learning Tool
//
//  Created by Luong Nhat Khoi on 1/12/25.
//

#include "AlgorithmVisualizer.h"

AlgorithmVisualizer::AlgorithmVisualizer()
    : currentStep(-1), currentAlgorithm(AlgorithmType::NONE),
      running(false), autoPlay(false), playSpeed(1.0f),
      bipartiteResult(false) {}

// Run Algorithms 

void AlgorithmVisualizer::runBFS(Graph& graph, int startNode) {
    clear();
    steps = graph.bfs(startNode);
    currentAlgorithm = AlgorithmType::BFS;
    running = !steps.empty();
    currentStep = 0;
    lastStepTime = std::chrono::steady_clock::now();
}

void AlgorithmVisualizer::runDFS(Graph& graph, int startNode) {
    clear();
    steps = graph.dfs(startNode);
    currentAlgorithm = AlgorithmType::DFS;
    running = !steps.empty();
    currentStep = 0;
    lastStepTime = std::chrono::steady_clock::now();
}

void AlgorithmVisualizer::runDijkstra(Graph& graph, int startNode, int endNode) {
    clear();
    steps = graph.dijkstra(startNode, endNode);
    currentAlgorithm = AlgorithmType::DIJKSTRA;
    running = !steps.empty();
    currentStep = 0;
    lastStepTime = std::chrono::steady_clock::now();
}

void AlgorithmVisualizer::runBipartiteCheck(Graph& graph) {
    clear();
    
    auto [isBipartite, coloring] = graph.checkBipartite();
    bipartiteResult = isBipartite;
    bipartiteColoring = coloring;
    
    // Create visualization step
    AlgorithmStep step;
    if (isBipartite) {
        step.description = "Graph IS bipartite! Nodes are 2-colored.";
        for (const auto& node : graph.getNodes()) {
            if (coloring[node.id] == 0) {
                step.visitedNodes.push_back(node.id);  // Color 0 - visited
            } else {
                step.highlightedNodes.push_back(node.id);  // Color 1 - highlighted
            }
        }
    } else {
        step.description = "Graph is NOT bipartite. Cannot 2-color the graph.";
        for (const auto& node : graph.getNodes()) {
            step.highlightedNodes.push_back(node.id);
        }
    }
    
    steps.push_back(step);
    currentAlgorithm = AlgorithmType::BIPARTITE;
    running = true;
    currentStep = 0;
}

void AlgorithmVisualizer::runPrimMST(Graph& graph, int startNode) {
    clear();
    steps = graph.primMST(startNode);
    currentAlgorithm = AlgorithmType::PRIM_MST;
    running = !steps.empty();
    currentStep = 0;
    lastStepTime = std::chrono::steady_clock::now();
}

void AlgorithmVisualizer::runKruskalMST(Graph& graph) {
    clear();
    steps = graph.kruskalMST();
    currentAlgorithm = AlgorithmType::KRUSKAL_MST;
    running = !steps.empty();
    currentStep = 0;
    lastStepTime = std::chrono::steady_clock::now();
}

void AlgorithmVisualizer::runFordFulkerson(Graph& graph, int source, int sink) {
    clear();
    steps = graph.fordFulkerson(source, sink);
    currentAlgorithm = AlgorithmType::FORD_FULKERSON;
    running = !steps.empty();
    currentStep = 0;
    lastStepTime = std::chrono::steady_clock::now();
}

void AlgorithmVisualizer::runFleuryEulerian(Graph& graph, int startNode) {
    clear();
    steps = graph.fleuryEulerian(startNode);
    currentAlgorithm = AlgorithmType::FLEURY_EULERIAN;
    running = !steps.empty();
    currentStep = 0;
    lastStepTime = std::chrono::steady_clock::now();
}

void AlgorithmVisualizer::runHierholzerEulerian(Graph& graph, int startNode) {
    clear();
    steps = graph.hierholzerEulerian(startNode);
    currentAlgorithm = AlgorithmType::HIERHOLZER_EULERIAN;
    running = !steps.empty();
    currentStep = 0;
    lastStepTime = std::chrono::steady_clock::now();
}

// Playback Control

void AlgorithmVisualizer::nextStep() {
    if (currentStep < (int)steps.size() - 1) {
        currentStep++;
        lastStepTime = std::chrono::steady_clock::now();
    }
}

void AlgorithmVisualizer::prevStep() {
    if (currentStep > 0) {
        currentStep--;
        lastStepTime = std::chrono::steady_clock::now();
    }
}

void AlgorithmVisualizer::reset() {
    currentStep = 0;
    lastStepTime = std::chrono::steady_clock::now();
}

void AlgorithmVisualizer::setAutoPlay(bool play, float speed) {
    autoPlay = play;
    playSpeed = speed;
    lastStepTime = std::chrono::steady_clock::now();
}

void AlgorithmVisualizer::update() {
    if (!autoPlay || !running || isFinished()) return;
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastStepTime);
    
    if (elapsed.count() >= 1000 / playSpeed) {
        nextStep();
    }
}

// State Getters

bool AlgorithmVisualizer::isRunning() const {
    return running;
}

bool AlgorithmVisualizer::isFinished() const {
    return running && currentStep >= (int)steps.size() - 1;
}

int AlgorithmVisualizer::getCurrentStepIndex() const {
    return currentStep;
}

int AlgorithmVisualizer::getTotalSteps() const {
    return steps.size();
}

const AlgorithmStep& AlgorithmVisualizer::getCurrentStep() const {
    if (currentStep >= 0 && currentStep < (int)steps.size()) {
        return steps[currentStep];
    }
    return emptyStep;
}

AlgorithmType AlgorithmVisualizer::getCurrentAlgorithm() const {
    return currentAlgorithm;
}

std::string AlgorithmVisualizer::getCurrentDescription() const {
    if (currentStep >= 0 && currentStep < (int)steps.size()) {
        return steps[currentStep].description;
    }
    return "";
}

// Visualization Data Getters

const std::vector<int>& AlgorithmVisualizer::getHighlightedNodes() const {
    if (currentStep >= 0 && currentStep < (int)steps.size()) {
        return steps[currentStep].highlightedNodes;
    }
    return emptyStep.highlightedNodes;
}

const std::vector<std::pair<int,int>>& AlgorithmVisualizer::getHighlightedEdges() const {
    if (currentStep >= 0 && currentStep < (int)steps.size()) {
        return steps[currentStep].highlightedEdges;
    }
    return emptyStep.highlightedEdges;
}

const std::vector<int>& AlgorithmVisualizer::getVisitedNodes() const {
    if (currentStep >= 0 && currentStep < (int)steps.size()) {
        return steps[currentStep].visitedNodes;
    }
    return emptyStep.visitedNodes;
}

const std::vector<std::pair<int,int>>& AlgorithmVisualizer::getVisitedEdges() const {
    if (currentStep >= 0 && currentStep < (int)steps.size()) {
        return steps[currentStep].visitedEdges;
    }
    return emptyStep.visitedEdges;
}

const std::vector<int>& AlgorithmVisualizer::getPath() const {
    if (currentStep >= 0 && currentStep < (int)steps.size()) {
        return steps[currentStep].path;
    }
    return emptyStep.path;
}

const std::map<int, double>& AlgorithmVisualizer::getNodeValues() const {
    if (currentStep >= 0 && currentStep < (int)steps.size()) {
        return steps[currentStep].nodeValues;
    }
    return emptyStep.nodeValues;
}

// Clear

void AlgorithmVisualizer::clear() {
    steps.clear();
    currentStep = -1;
    currentAlgorithm = AlgorithmType::NONE;
    running = false;
    autoPlay = false;
    bipartiteResult = false;
    bipartiteColoring.clear();
}
