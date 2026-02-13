//
//  GraphRenderer.h
//  Graph Visualization - Data Structures & Algorithms Learning Tool
//
//  Created by Luong Nhat Khoi on 1/12/25.
//

#pragma once
#include <SFML/Graphics.hpp>
#include "Graph.h"
#include "AlgorithmVisualizer.h"
#include <set>

class GraphRenderer {
public:
    GraphRenderer(sf::RenderWindow& window);
    void render(const Graph& graph, const AlgorithmVisualizer* visualizer = nullptr);
    
    void setSelectedNode(int id);
    void setSecondSelectedNode(int id);
    int getSelectedNode() const;
    int getSecondSelectedNode() const;
    
    void setShowWeights(bool show);
    void setShowNodeIds(bool show);
    void setShowDistances(bool show);
    bool getShowWeights() const;
    
    void startDragging(int nodeId, float mouseX, float mouseY);
    void updateDragging(float mouseX, float mouseY, Graph& graph);
    void stopDragging();
    bool isDragging() const;
    
    void setEditingEdgeWeight(int u, int v);
    void updateEdgeWeightInput(char c, Graph& graph);
    void confirmEdgeWeight(Graph& graph);
    bool isEditingEdgeWeight() const;
    std::pair<int,int> getEditingEdge() const;

private:
    sf::RenderWindow& window;
    sf::Font font;
    
    int selectedNodeId = -1;
    int secondSelectedNodeId = -1;
    
    bool showWeights = true;
    bool showNodeIds = true;
    bool showDistances = false;
    
    bool dragging = false;
    int dragNodeId = -1;
    float dragOffsetX = 0, dragOffsetY = 0;

    bool editingWeight = false;
    int editEdgeU = -1, editEdgeV = -1;
    std::string weightInputBuffer;
    
    sf::Color nodeDefaultColor = sf::Color::White;
    sf::Color nodeSelectedColor = sf::Color::Green;
    sf::Color nodeVisitedColor = sf::Color(100, 200, 255);
    sf::Color nodeHighlightColor = sf::Color::Yellow;
    sf::Color nodePathColor = sf::Color(255, 100, 100);
    sf::Color nodeBipartite0 = sf::Color(255, 150, 150);
    sf::Color nodeBipartite1 = sf::Color(150, 150, 255);
    
    sf::Color edgeDefaultColor = sf::Color(150, 150, 150);
    sf::Color edgeVisitedColor = sf::Color(100, 200, 255);
    sf::Color edgeHighlightColor = sf::Color::Yellow;
    sf::Color edgePathColor = sf::Color(255, 100, 100);
    
    void drawNode(const Node& node, sf::Color fillColor, sf::Color outlineColor, 
                  const std::map<int, double>* nodeValues = nullptr);
    void drawEdge(const Edge& edge, const std::vector<Node>& nodes, 
                  sf::Color color, float thickness = 2.0f);
    void drawArrow(sf::Vector2f from, sf::Vector2f to, sf::Color color, float thickness);
    void drawWeightLabel(const Edge& edge, const std::vector<Node>& nodes);
};
