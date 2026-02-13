//
//  FileIO.h
//  Graph Visualization - Data Structures & Algorithms Learning Tool
//
//  Created by Luong Nhat Khoi on 1/12/25.
//

#pragma once
#include "Graph.h"
#include <string>
#include <fstream>
#include <sstream>

class FileIO {
public:
    static bool saveGraph(const Graph& graph, const std::string& filename);
    
    static bool loadGraph(Graph& graph, const std::string& filename);
    
    static bool exportAdjacencyMatrix(const Graph& graph, const std::string& filename);
    static bool exportAdjacencyList(const Graph& graph, const std::string& filename);
    static bool exportEdgeList(const Graph& graph, const std::string& filename);
    
    static bool importAdjacencyMatrix(Graph& graph, const std::string& filename, bool isDirected = false);
    static bool importEdgeList(Graph& graph, const std::string& filename, bool isDirected = false);
    
    static std::string getAdjacencyMatrixString(const Graph& graph);
    static std::string getAdjacencyListString(const Graph& graph);
    static std::string getEdgeListString(const Graph& graph);
};
