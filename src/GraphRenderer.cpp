//
//  GraphRenderer.cpp
//  Graph Visualization - Data Structures & Algorithms Learning Tool
//
//  Created by Luong Nhat Khoi on 1/12/25.
//

#include "GraphRenderer.h"
#include <string>
#include <cmath>
#include <algorithm>
#include <vector>

GraphRenderer::GraphRenderer(sf::RenderWindow& win) : window(win) {
    std::vector<std::string> fontPaths = {
        "resources/fonts/DejaVuSans.ttf",
        "../resources/fonts/DejaVuSans.ttf",
        "resources/fonts/Arial.ttf",
        "../resources/fonts/Arial.ttf",
        
        // macOS system fonts
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/System/Library/Fonts/Helvetica.ttc",
        "/Library/Fonts/Arial.ttf",
        
        // Linux system fonts
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
        
        // Windows system fonts
        "C:/Windows/Fonts/arial.ttf",
        "C:/Windows/Fonts/segoeui.ttf",
        "C:/Windows/Fonts/tahoma.ttf"
    };
    
    bool fontLoaded = false;
    for (const auto& path : fontPaths) {
        if (font.openFromFile(path)) {  
            fontLoaded = true;
            break;
        }
    }
    
    if (!fontLoaded) {
        throw std::runtime_error(
            "Could not load any font! Please place DejaVuSans.ttf in resources/fonts/");
    }
    
    font.setSmooth(true);
}

void GraphRenderer::render(const Graph& graph, const AlgorithmVisualizer* visualizer) {
    const auto& nodes = graph.getNodes();
    const auto& edges = graph.getEdges();
    std::set<int> visitedNodes, highlightedNodes, pathNodes;
    std::set<std::pair<int,int>> visitedEdges, highlightedEdges;
    const std::map<int, double>* nodeValues = nullptr;
    
    if (visualizer && visualizer->isRunning()) {
        for (int n : visualizer->getVisitedNodes()) visitedNodes.insert(n);
        for (int n : visualizer->getHighlightedNodes()) highlightedNodes.insert(n);
        for (int n : visualizer->getPath()) pathNodes.insert(n);
        
        for (const auto& e : visualizer->getVisitedEdges()) {
            visitedEdges.insert(e);
            visitedEdges.insert({e.second, e.first});
        }
        for (const auto& e : visualizer->getHighlightedEdges()) {
            highlightedEdges.insert(e);
            highlightedEdges.insert({e.second, e.first});
        }
        
        if (!visualizer->getNodeValues().empty()) {
            nodeValues = &visualizer->getNodeValues();
        }
    }

    for (const auto& edge : edges) {
        sf::Color edgeColor = edgeDefaultColor;
        float thickness = 2.0f;
        
        auto edgePair = std::make_pair(edge.source, edge.target);
        auto reverseEdge = std::make_pair(edge.target, edge.source);
        
        if (highlightedEdges.count(edgePair) || highlightedEdges.count(reverseEdge)) {
            edgeColor = edgeHighlightColor;
            thickness = 4.0f;
        } else if (visitedEdges.count(edgePair) || visitedEdges.count(reverseEdge)) {
            edgeColor = edgeVisitedColor;
            thickness = 3.0f;
        }

        if (visualizer && visualizer->isRunning()) {
            const auto& path = visualizer->getPath();
            for (size_t i = 0; i + 1 < path.size(); i++) {
                if ((path[i] == edge.source && path[i+1] == edge.target) ||
                    (path[i] == edge.target && path[i+1] == edge.source)) {
                    edgeColor = edgePathColor;
                    thickness = 4.0f;
                    break;
                }
            }
        }
        
        drawEdge(edge, nodes, edgeColor, thickness);
        
        if (showWeights && edge.weight != 1.0) {
            drawWeightLabel(edge, nodes);
        }
    }

    for (const auto& node : nodes) {
        sf::Color fillColor = nodeDefaultColor;
        sf::Color outlineColor = sf::Color(0, 100, 255);
        
        if (node.id == selectedNodeId) {
            fillColor = nodeSelectedColor;
            outlineColor = sf::Color::Yellow;
        } else if (node.id == secondSelectedNodeId) {
            fillColor = sf::Color(255, 165, 0); 
            outlineColor = sf::Color::Yellow;
        } else if (pathNodes.count(node.id)) {
            fillColor = nodePathColor;
            outlineColor = sf::Color::Red;
        } else if (highlightedNodes.count(node.id)) {
            fillColor = nodeHighlightColor;
            outlineColor = sf::Color(200, 200, 0);
        } else if (visitedNodes.count(node.id)) {
            fillColor = nodeVisitedColor;
            outlineColor = sf::Color(0, 150, 200);
        }

        if (visualizer && visualizer->getCurrentAlgorithm() == AlgorithmType::BIPARTITE) {
            if (node.color == 0) {
                fillColor = nodeBipartite0;
                outlineColor = sf::Color(200, 100, 100);
            } else if (node.color == 1) {
                fillColor = nodeBipartite1;
                outlineColor = sf::Color(100, 100, 200);
            }
        }
        
        drawNode(node, fillColor, outlineColor, nodeValues);
    }
    
    if (editingWeight) {
        sf::RectangleShape editBox({80, 30});
        editBox.setPosition({window.getSize().x / 2.0f - 40, 50});
        editBox.setFillColor(sf::Color::White);
        editBox.setOutlineColor(sf::Color::Black);
        editBox.setOutlineThickness(2);
        window.draw(editBox);
        
        sf::Text editText(font, "Weight: " + weightInputBuffer, 16);
        editText.setFillColor(sf::Color::Black);
        editText.setPosition({window.getSize().x / 2.0f - 35, 55});
        window.draw(editText);
    }
}

void GraphRenderer::drawNode(const Node& node, sf::Color fillColor, sf::Color outlineColor,
                             const std::map<int, double>* nodeValues) {
    float r = 25.0f;
    sf::CircleShape circle(r);
    circle.setOrigin({r, r});
    circle.setPosition({node.x, node.y});
    circle.setFillColor(fillColor);
    circle.setOutlineColor(outlineColor);
    circle.setOutlineThickness(3.0f);
    
    window.draw(circle);
    
    // Draw node ID with improved text rendering
    if (showNodeIds) {
        // Use larger font size for better clarity
        unsigned int fontSize = 20;
        sf::Text text(font, std::to_string(node.id), fontSize);
        text.setFillColor(sf::Color::Black);
        text.setStyle(sf::Text::Bold);
        
        // Add outline for better visibility
        text.setOutlineColor(sf::Color(255, 255, 255, 100));
        text.setOutlineThickness(1.0f);
        
        sf::FloatRect b = text.getLocalBounds();
        text.setOrigin({b.position.x + b.size.x/2, b.position.y + b.size.y/2});
        text.setPosition({node.x, node.y});
        window.draw(text);
    }
    
    // Draw distance value below node if available
    if (nodeValues && showDistances) {
        auto it = nodeValues->find(node.id);
        if (it != nodeValues->end()) {
            std::string distStr = (it->second >= 1e9) ? "INF" : std::to_string((int)it->second);
            unsigned int fontSize = 16;
            sf::Text distText(font, distStr, fontSize);
            distText.setFillColor(sf::Color::White);
            distText.setStyle(sf::Text::Bold);
            
            // Add dark outline for contrast
            distText.setOutlineColor(sf::Color::Black);
            distText.setOutlineThickness(1.5f);
            
            sf::FloatRect db = distText.getLocalBounds();
            distText.setOrigin({db.position.x + db.size.x/2, db.position.y + db.size.y/2});
            distText.setPosition({node.x, node.y + r + 15});
            window.draw(distText);
        }
    }
}

void GraphRenderer::drawEdge(const Edge& edge, const std::vector<Node>& nodes, 
                             sf::Color color, float thickness) {
    sf::Vector2f p1, p2;
    bool f1 = false, f2 = false;
    
    for (const auto& n : nodes) {
        if (n.id == edge.source) { p1 = {n.x, n.y}; f1 = true; }
        if (n.id == edge.target) { p2 = {n.x, n.y}; f2 = true; }
    }
    
    if (!f1 || !f2) return;
    
    if (edge.directed) {
        // Draw arrow for directed edge
        drawArrow(p1, p2, color, thickness);
    } else {
        // Draw line for undirected edge
        sf::Vector2f direction = p2 - p1;
        float length = std::sqrt(direction.x * direction.x + direction.y * direction.y);
        if (length == 0) return;
        
        sf::Vector2f unitDir = direction / length;
        sf::Vector2f perpendicular(-unitDir.y, unitDir.x);
        
        // Create a rectangle as a thick line
        sf::ConvexShape line;
        line.setPointCount(4);
        line.setPoint(0, p1 + perpendicular * (thickness / 2));
        line.setPoint(1, p1 - perpendicular * (thickness / 2));
        line.setPoint(2, p2 - perpendicular * (thickness / 2));
        line.setPoint(3, p2 + perpendicular * (thickness / 2));
        line.setFillColor(color);
        
        window.draw(line);
    }
}

void GraphRenderer::drawArrow(sf::Vector2f from, sf::Vector2f to, sf::Color color, float thickness) {
    sf::Vector2f direction = to - from;
    float length = std::sqrt(direction.x * direction.x + direction.y * direction.y);
    if (length == 0) return;
    
    sf::Vector2f unitDir = direction / length;
    sf::Vector2f perpendicular(-unitDir.y, unitDir.x);
    
    // Shorten line to not overlap with node
    float nodeRadius = 25.0f;
    sf::Vector2f adjustedTo = to - unitDir * nodeRadius;
    sf::Vector2f adjustedFrom = from + unitDir * nodeRadius;
    
    // Draw line body
    sf::ConvexShape line;
    line.setPointCount(4);
    line.setPoint(0, adjustedFrom + perpendicular * (thickness / 2));
    line.setPoint(1, adjustedFrom - perpendicular * (thickness / 2));
    line.setPoint(2, adjustedTo - perpendicular * (thickness / 2));
    line.setPoint(3, adjustedTo + perpendicular * (thickness / 2));
    line.setFillColor(color);
    window.draw(line);
    
    // Draw arrowhead
    float arrowSize = 12.0f;
    sf::ConvexShape arrow;
    arrow.setPointCount(3);
    arrow.setPoint(0, adjustedTo);
    arrow.setPoint(1, adjustedTo - unitDir * arrowSize + perpendicular * (arrowSize / 2));
    arrow.setPoint(2, adjustedTo - unitDir * arrowSize - perpendicular * (arrowSize / 2));
    arrow.setFillColor(color);
    window.draw(arrow);
}

void GraphRenderer::drawWeightLabel(const Edge& edge, const std::vector<Node>& nodes) {
    sf::Vector2f p1, p2;
    
    for (const auto& n : nodes) {
        if (n.id == edge.source) p1 = {n.x, n.y};
        if (n.id == edge.target) p2 = {n.x, n.y};
    }
    
    // Position label at midpoint, slightly offset
    sf::Vector2f mid = (p1 + p2) / 2.0f;
    sf::Vector2f dir = p2 - p1;
    float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
    if (len > 0) {
        sf::Vector2f perp(-dir.y / len, dir.x / len);
        mid += perp * 15.0f;
    }
    
    // Draw background with rounded appearance
    sf::RectangleShape bg({34, 22});
    bg.setOrigin({17, 11});
    bg.setPosition(mid);
    bg.setFillColor(sf::Color(40, 40, 40, 230));
    bg.setOutlineColor(sf::Color(100, 100, 100));
    bg.setOutlineThickness(1.0f);
    window.draw(bg);
    
    // Draw weight text with better clarity
    std::string weightStr = std::to_string((int)edge.weight);
    unsigned int fontSize = 16;
    sf::Text text(font, weightStr, fontSize);
    text.setFillColor(sf::Color::White);
    text.setStyle(sf::Text::Bold);
    
    // Add subtle outline for sharpness
    text.setOutlineColor(sf::Color::Black);
    text.setOutlineThickness(0.5f);
    
    sf::FloatRect b = text.getLocalBounds();
    text.setOrigin({b.position.x + b.size.x/2, b.position.y + b.size.y/2});
    text.setPosition(mid);
    window.draw(text);
}

// ==================== Selection ====================

void GraphRenderer::setSelectedNode(int id) {
    selectedNodeId = id;
}

void GraphRenderer::setSecondSelectedNode(int id) {
    secondSelectedNodeId = id;
}

int GraphRenderer::getSelectedNode() const {
    return selectedNodeId;
}

int GraphRenderer::getSecondSelectedNode() const {
    return secondSelectedNodeId;
}

// ==================== Display Options ====================

void GraphRenderer::setShowWeights(bool show) {
    showWeights = show;
}

void GraphRenderer::setShowNodeIds(bool show) {
    showNodeIds = show;
}

void GraphRenderer::setShowDistances(bool show) {
    showDistances = show;
}

bool GraphRenderer::getShowWeights() const {
    return showWeights;
}

// ==================== Dragging ====================

void GraphRenderer::startDragging(int nodeId, float mouseX, float mouseY) {
    dragNodeId = nodeId;
    dragging = true;
    dragOffsetX = mouseX;
    dragOffsetY = mouseY;
}

void GraphRenderer::updateDragging(float mouseX, float mouseY, Graph& graph) {
    if (!dragging) return;
    
    Node* node = graph.getNodeById(dragNodeId);
    if (node) {
        node->x = mouseX;
        node->y = mouseY;
    }
}

void GraphRenderer::stopDragging() {
    dragging = false;
    dragNodeId = -1;
}

bool GraphRenderer::isDragging() const {
    return dragging;
}

// ==================== Edge Weight Editing ====================

void GraphRenderer::setEditingEdgeWeight(int u, int v) {
    editEdgeU = u;
    editEdgeV = v;
    editingWeight = true;
    weightInputBuffer = "";
}

void GraphRenderer::updateEdgeWeightInput(char c, Graph& graph) {
    if (!editingWeight) return;
    
    if (c == '\b' && !weightInputBuffer.empty()) {
        weightInputBuffer.pop_back();
    } else if (c >= '0' && c <= '9') {
        weightInputBuffer += c;
    }
}

void GraphRenderer::confirmEdgeWeight(Graph& graph) {
    if (!editingWeight || weightInputBuffer.empty()) {
        editingWeight = false;
        return;
    }
    
    try {
        double weight = std::stod(weightInputBuffer);
        graph.setEdgeWeight(editEdgeU, editEdgeV, weight);
    } catch (...) {}
    
    editingWeight = false;
    weightInputBuffer = "";
}

bool GraphRenderer::isEditingEdgeWeight() const {
    return editingWeight;
}

std::pair<int,int> GraphRenderer::getEditingEdge() const {
    return {editEdgeU, editEdgeV};
}
