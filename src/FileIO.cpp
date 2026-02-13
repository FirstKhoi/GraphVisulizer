//
//  FileIO.cpp
//  Graph Visualization - Data Structures & Algorithms Learning Tool
//
//  Created by Luong Nhat Khoi on 1/12/25.
//

#include "FileIO.h"
#include <iomanip>

//  Save/Load Graph (Custom Format) 

bool FileIO::saveGraph(const Graph& graph, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    // Write header
    file << "GRAPH_FILE_V1\n";
    file << "DIRECTED " << (graph.isDirectedGraph() ? "1" : "0") << "\n";
    
    // Write nodes
    file << "NODES " << graph.getNodes().size() << "\n";
    for (const auto& node : graph.getNodes()) {
        file << node.id << " " << node.x << " " << node.y << "\n";
    }
    
    // Write edges
    file << "EDGES " << graph.getEdges().size() << "\n";
    for (const auto& edge : graph.getEdges()) {
        file << edge.source << " " << edge.target << " " << edge.weight << "\n";
    }
    
    file.close();
    return true;
}

bool FileIO::loadGraph(Graph& graph, const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) return false;
    
    graph.clear();
    
    std::string line, token;
    
    std::getline(file, line);
    if (line != "GRAPH_FILE_V1") return false;
    
    std::getline(file, line);
    std::istringstream dirStream(line);
    dirStream >> token;
    if (token != "DIRECTED") return false;
    int isDir;
    dirStream >> isDir;
    graph.setDirected(isDir == 1);
    
    std::getline(file, line);
    std::istringstream nodeStream(line);
    nodeStream >> token;
    if (token != "NODES") return false;
    int nodeCount;
    nodeStream >> nodeCount;
    
    for (int i = 0; i < nodeCount; i++) {
        int id;
        float x, y;
        file >> id >> x >> y;
        graph.addNode(x, y);
    }
    
    // Read edges
    file.ignore();
    std::getline(file, line);
    std::istringstream edgeStream(line);
    edgeStream >> token;
    if (token != "EDGES") return false;
    int edgeCount;
    edgeStream >> edgeCount;
    
    for (int i = 0; i < edgeCount; i++) {
        int source, target;
        double weight;
        file >> source >> target >> weight;
        graph.addEdge(source, target, weight, graph.isDirectedGraph());
    }
    
    file.close();
    return true;
}


bool FileIO::exportAdjacencyMatrix(const Graph& graph, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    file << getAdjacencyMatrixString(graph);
    file.close();
    return true;
}

bool FileIO::exportAdjacencyList(const Graph& graph, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    file << getAdjacencyListString(graph);
    file.close();
    return true;
}

bool FileIO::exportEdgeList(const Graph& graph, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    file << getEdgeListString(graph);
    file.close();
    return true;
}

//  Import from Representations 

bool FileIO::importAdjacencyMatrix(Graph& graph, const std::string& filename, bool isDirected) {
    std::ifstream file(filename);
    if (!file.is_open()) return false;
    
    std::vector<std::vector<double>> matrix;
    std::string line;
    
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::vector<double> row;
        double val;
        while (iss >> val) {
            row.push_back(val);
        }
        if (!row.empty()) {
            matrix.push_back(row);
        }
    }
    
    if (matrix.empty()) return false;
    
    graph.loadFromAdjacencyMatrix(matrix, isDirected);
    file.close();
    return true;
}

bool FileIO::importEdgeList(Graph& graph, const std::string& filename, bool isDirected) {
    std::ifstream file(filename);
    if (!file.is_open()) return false;
    
    std::vector<std::tuple<int, int, double>> edgeList;
    std::string line;
    
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        int u, v;
        double w = 1.0;
        if (iss >> u >> v) {
            iss >> w; 
            edgeList.push_back({u, v, w});
        }
    }
    
    if (edgeList.empty()) return false;
    
    graph.loadFromEdgeList(edgeList, isDirected);
    file.close();
    return true;
}


std::string FileIO::getAdjacencyMatrixString(const Graph& graph) {
    std::ostringstream oss;
    
    auto matrix = graph.getAdjacencyMatrix();
    auto nodes = graph.getNodes();
    
    if (matrix.empty()) {
        return "Empty graph";
    }
    
    // Create ID mapping for display
    std::vector<int> ids;
    for (const auto& n : nodes) {
        ids.push_back(n.id);
    }
    std::sort(ids.begin(), ids.end());
    
    // Header row
    oss << "    ";
    for (int id : ids) {
        oss << std::setw(4) << id;
    }
    oss << "\n";
    
    // Matrix rows
    for (size_t i = 0; i < matrix.size(); i++) {
        oss << std::setw(4) << ids[i];
        for (size_t j = 0; j < matrix[i].size(); j++) {
            if (matrix[i][j] == 0) {
                oss << std::setw(4) << "0";
            } else {
                oss << std::setw(4) << (int)matrix[i][j];
            }
        }
        oss << "\n";
    }
    
    return oss.str();
}

std::string FileIO::getAdjacencyListString(const Graph& graph) {
    std::ostringstream oss;
    
    auto adjList = graph.getAdjacencyList();
    
    if (adjList.empty()) {
        return "Empty graph";
    }
    
    for (const auto& [node, neighbors] : adjList) {
        oss << node << " -> ";
        for (size_t i = 0; i < neighbors.size(); i++) {
            oss << neighbors[i].first;
            if (neighbors[i].second != 1.0) {
                oss << "(" << (int)neighbors[i].second << ")";
            }
            if (i < neighbors.size() - 1) oss << ", ";
        }
        if (neighbors.empty()) oss << "(none)";
        oss << "\n";
    }
    
    return oss.str();
}

std::string FileIO::getEdgeListString(const Graph& graph) {
    std::ostringstream oss;
    
    auto edgeList = graph.getEdgeList();
    
    if (edgeList.empty()) {
        return "Empty graph (no edges)";
    }
    
    oss << "Edges:\n";
    for (const auto& [u, v, w] : edgeList) {
        oss << "(" << u << ", " << v << ")";
        if (w != 1.0) {
            oss << " weight: " << (int)w;
        }
        oss << "\n";
    }
    
    return oss.str();
}
