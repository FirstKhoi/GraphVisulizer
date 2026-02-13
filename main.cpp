//
//  main.cpp
//  Graph Visualization - Data Structures & Algorithms Learning Tool
//
//  Created by Luong Nhat Khoi on 1/12/25.
//
//  Features:
//  - Visual graph drawing (add/remove vertices and edges)
//  - Directed and undirected graphs with weights
//  - Save and load graphs
//  - BFS, DFS traversal visualization
//  - Dijkstra's shortest path
//  - Bipartite check
//  - Prim's and Kruskal's MST
//  - Ford-Fulkerson max flow
//  - Fleury's and Hierholzer's Eulerian path/circuit
//  - Graph representation conversion (matrix, list, edge list)
//

#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <sstream>
#include "include/Graph.h"
#include "include/GraphRenderer.h"
#include "include/Button.h"
#include "include/AlgorithmVisualizer.h"
#include "include/FileIO.h"

// Application modes
enum class AppMode {
    EDITOR,              // Default editing mode
    SELECT_START,        // Selecting start node for algorithms
    SELECT_END,          // Selecting end node (for Dijkstra, Ford-Fulkerson)
    RUNNING_ALGORITHM,   // Algorithm visualization in progress
    VIEW_REPRESENTATION, // Viewing graph representations
    INPUT_WEIGHT         // Inputting edge weight
};

// Algorithm requiring selection
enum class PendingAlgorithm {
    NONE,
    BFS,
    DFS,
    DIJKSTRA,
    PRIM,
    KRUSKAL,
    FORD_FULKERSON,
    FLEURY,
    HIERHOLZER,
    BIPARTITE
};

int main() {
    // Retina display settings
    sf::ContextSettings settings;
    settings.antiAliasingLevel = 8;  // Fix: capital A in AntiAliasing

    sf::RenderWindow window(
        sf::VideoMode({1200, 800}),
        "Graph Visualization - DSA Learning Tool",
        sf::Style::Default,
        sf::State::Windowed,
        settings
    );
    window.setFramerateLimit(60);
    window.setVerticalSyncEnabled(true); // Enable VSync for smoother rendering

    sf::Font font;
    if (!font.openFromFile("arial.ttf")) {
        if (!font.openFromFile("ARIAL.TTF")) {
            if (!font.openFromFile("C:/Windows/Fonts/arial.ttf")) {  // Windows
                if (!font.openFromFile("/System/Library/Fonts/Helvetica.ttc")) {  // macOS
                    if (!font.openFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")) {  // Linux
                        std::cerr << "Error: Could not load font!\n";
                        return 1;
                    }
                }
            }
        }
    }

    Graph graph;
    GraphRenderer renderer(window);
    AlgorithmVisualizer visualizer;
    
    AppMode currentMode = AppMode::EDITOR;
    PendingAlgorithm pendingAlgo = PendingAlgorithm::NONE;
    int selectedStartNode = -1;
    int selectedEndNode = -1;
    
    // Edge weight input
    std::string weightInput = "";
    int pendingEdgeFrom = -1;
    int pendingEdgeTo = -1;
    double defaultWeight = 1.0;

    const float MENU_WIDTH = 220.0f;
    const float BOTTOM_PANEL_HEIGHT = 150.0f;
    const float WINDOW_HEIGHT = 800.0f;  // Add this constant
    const float WINDOW_WIDTH = 1200.0f;  // Add this constant
    const float BTN_WIDTH = 200.0f;
    const float BTN_HEIGHT = 32.0f;
    const float BTN_GAP = 36.0f;
    const float BTN_X = 10.0f;
    
    sf::FloatRect graphArea({MENU_WIDTH, 0}, {WINDOW_WIDTH - MENU_WIDTH, WINDOW_HEIGHT - BOTTOM_PANEL_HEIGHT});
    
    std::vector<Button> buttons;
    float btnY = 10.0f;
    
    
    sf::Text editLabel(font, "EDIT TOOLS", 14);
    editLabel.setFillColor(sf::Color::Cyan);
    editLabel.setPosition({BTN_X, btnY});
    btnY += 25;
    
    buttons.emplace_back(BTN_X, btnY, BTN_WIDTH, BTN_HEIGHT, "Toggle Directed", font);
    buttons.back().onClick = [&]() {
        graph.setDirected(!graph.isDirectedGraph());
        std::cout << "Graph is now " << (graph.isDirectedGraph() ? "DIRECTED" : "UNDIRECTED") << "\n";
    };
    btnY += BTN_GAP;
    
    buttons.emplace_back(BTN_X, btnY, BTN_WIDTH, BTN_HEIGHT, "Toggle Weights", font);
    buttons.back().onClick = [&]() {
        renderer.setShowWeights(!renderer.getShowWeights());
    };
    btnY += BTN_GAP;
    
    buttons.emplace_back(BTN_X, btnY, BTN_WIDTH, BTN_HEIGHT, "Clear Graph", font);
    buttons.back().onClick = [&]() {
        graph.clear();
        visualizer.clear();
        currentMode = AppMode::EDITOR;
    };
    btnY += BTN_GAP + 10;
    
    //I/O
    sf::Text fileLabel(font, "FILE I/O", 14);
    fileLabel.setFillColor(sf::Color::Cyan);
    fileLabel.setPosition({BTN_X, btnY});
    btnY += 25;
    
    buttons.emplace_back(BTN_X, btnY, BTN_WIDTH, BTN_HEIGHT, "Save Graph", font);
    buttons.back().onClick = [&]() {
        if (FileIO::saveGraph(graph, "graph.txt")) {
            std::cout << "Graph saved to graph.txt\n";
        }
    };
    btnY += BTN_GAP;
    
    buttons.emplace_back(BTN_X, btnY, BTN_WIDTH, BTN_HEIGHT, "Load Graph", font);
    buttons.back().onClick = [&]() {
        if (FileIO::loadGraph(graph, "graph.txt")) {
            std::cout << "Graph loaded from graph.txt\n";
            visualizer.clear();
        }
    };
    btnY += BTN_GAP + 10;
    
    sf::Text basicLabel(font, "BASIC ALGORITHMS", 14);
    basicLabel.setFillColor(sf::Color::Cyan);
    basicLabel.setPosition({BTN_X, btnY});
    btnY += 25;
    
    buttons.emplace_back(BTN_X, btnY, BTN_WIDTH, BTN_HEIGHT, "BFS Traversal", font);
    buttons.back().onClick = [&]() {
        if (graph.getNodeCount() == 0) return;
        currentMode = AppMode::SELECT_START;
        pendingAlgo = PendingAlgorithm::BFS;
        std::cout << "Click a node to start BFS\n";
    };
    btnY += BTN_GAP;
    
    buttons.emplace_back(BTN_X, btnY, BTN_WIDTH, BTN_HEIGHT, "DFS Traversal", font);
    buttons.back().onClick = [&]() {
        if (graph.getNodeCount() == 0) return;
        currentMode = AppMode::SELECT_START;
        pendingAlgo = PendingAlgorithm::DFS;
        std::cout << "Click a node to start DFS\n";
    };
    btnY += BTN_GAP;
    
    buttons.emplace_back(BTN_X, btnY, BTN_WIDTH, BTN_HEIGHT, "Dijkstra Shortest Path", font);
    buttons.back().onClick = [&]() {
        if (graph.getNodeCount() < 2) return;
        currentMode = AppMode::SELECT_START;
        pendingAlgo = PendingAlgorithm::DIJKSTRA;
        std::cout << "Click START node for Dijkstra\n";
    };
    btnY += BTN_GAP;
    
    buttons.emplace_back(BTN_X, btnY, BTN_WIDTH, BTN_HEIGHT, "Check Bipartite", font);
    buttons.back().onClick = [&]() {
        if (graph.getNodeCount() == 0) return;
        visualizer.runBipartiteCheck(graph);
        currentMode = AppMode::RUNNING_ALGORITHM;
    };
    btnY += BTN_GAP + 10;
    
    //===Advanced Algorithms===
    sf::Text advLabel(font, "ADVANCED ALGORITHMS", 14);
    advLabel.setFillColor(sf::Color::Cyan);
    advLabel.setPosition({BTN_X, btnY});
    btnY += 25;
    
    buttons.emplace_back(BTN_X, btnY, BTN_WIDTH, BTN_HEIGHT, "Prim's MST", font);
    buttons.back().onClick = [&]() {
        if (graph.getNodeCount() == 0) return;
        currentMode = AppMode::SELECT_START;
        pendingAlgo = PendingAlgorithm::PRIM;
        std::cout << "Click a node to start Prim's MST\n";
    };
    btnY += BTN_GAP;
    
    buttons.emplace_back(BTN_X, btnY, BTN_WIDTH, BTN_HEIGHT, "Kruskal's MST", font);
    buttons.back().onClick = [&]() {
        if (graph.getNodeCount() == 0) return;
        visualizer.runKruskalMST(graph);
        currentMode = AppMode::RUNNING_ALGORITHM;
    };
    btnY += BTN_GAP;
    
    buttons.emplace_back(BTN_X, btnY, BTN_WIDTH, BTN_HEIGHT, "Ford-Fulkerson Flow", font);
    buttons.back().onClick = [&]() {
        if (graph.getNodeCount() < 2) return;
        currentMode = AppMode::SELECT_START;
        pendingAlgo = PendingAlgorithm::FORD_FULKERSON;
        std::cout << "Click SOURCE node for Max Flow\n";
    };
    btnY += BTN_GAP;
    
    buttons.emplace_back(BTN_X, btnY, BTN_WIDTH, BTN_HEIGHT, "Fleury Eulerian", font);
    buttons.back().onClick = [&]() {
        if (graph.getNodeCount() == 0) return;
        currentMode = AppMode::SELECT_START;
        pendingAlgo = PendingAlgorithm::FLEURY;
        std::cout << "Click a node to start Fleury's algorithm\n";
    };
    btnY += BTN_GAP;
    
    buttons.emplace_back(BTN_X, btnY, BTN_WIDTH, BTN_HEIGHT, "Hierholzer Eulerian", font);
    buttons.back().onClick = [&]() {
        if (graph.getNodeCount() == 0) return;
        currentMode = AppMode::SELECT_START;
        pendingAlgo = PendingAlgorithm::HIERHOLZER;
        std::cout << "Click a node to start Hierholzer's algorithm\n";
    };
    btnY += BTN_GAP + 10;
    
    // === SECTION: Representations ===
    sf::Text repLabel(font, "REPRESENTATIONS", 14);
    repLabel.setFillColor(sf::Color::Cyan);
    repLabel.setPosition({BTN_X, btnY});
    btnY += 25;
    
    std::string currentRepresentation = "";
    
    buttons.emplace_back(BTN_X, btnY, BTN_WIDTH, BTN_HEIGHT, "Adjacency Matrix", font);
    buttons.back().onClick = [&]() {
        currentRepresentation = FileIO::getAdjacencyMatrixString(graph);
        currentMode = AppMode::VIEW_REPRESENTATION;
    };
    btnY += BTN_GAP;
    
    buttons.emplace_back(BTN_X, btnY, BTN_WIDTH, BTN_HEIGHT, "Adjacency List", font);
    buttons.back().onClick = [&]() {
        currentRepresentation = FileIO::getAdjacencyListString(graph);
        currentMode = AppMode::VIEW_REPRESENTATION;
    };
    btnY += BTN_GAP;
    
    buttons.emplace_back(BTN_X, btnY, BTN_WIDTH, BTN_HEIGHT, "Edge List", font);
    buttons.back().onClick = [&]() {
        currentRepresentation = FileIO::getEdgeListString(graph);
        currentMode = AppMode::VIEW_REPRESENTATION;
    };
    
    // Algorithm control buttons (bottom panel)
    std::vector<Button> controlButtons;
    float ctrlY = WINDOW_HEIGHT - BOTTOM_PANEL_HEIGHT + 10;  // Use constant
    
    controlButtons.emplace_back(MENU_WIDTH + 10, ctrlY, 100, 30, "< Prev", font);
    controlButtons.back().onClick = [&]() {
        visualizer.prevStep();
    };
    
    controlButtons.emplace_back(MENU_WIDTH + 120, ctrlY, 100, 30, "Next >", font);
    controlButtons.back().onClick = [&]() {
        visualizer.nextStep();
    };
    
    controlButtons.emplace_back(MENU_WIDTH + 230, ctrlY, 100, 30, "Reset", font);
    controlButtons.back().onClick = [&]() {
        visualizer.reset();
    };
    
    bool autoPlayEnabled = false;
    controlButtons.emplace_back(MENU_WIDTH + 340, ctrlY, 100, 30, "Auto Play", font);
    controlButtons.back().onClick = [&]() {
        autoPlayEnabled = !autoPlayEnabled;
        visualizer.setAutoPlay(autoPlayEnabled, 1.0f);
    };
    
    controlButtons.emplace_back(MENU_WIDTH + 450, ctrlY, 100, 30, "Stop", font);
    controlButtons.back().onClick = [&]() {
        visualizer.clear();
        currentMode = AppMode::EDITOR;
        renderer.setSelectedNode(-1);
        renderer.setSecondSelectedNode(-1);
    };
    
    // Main loop
    while (window.isOpen()) {
        sf::Vector2i mousePos = sf::Mouse::getPosition(window);
        float mx = (float)mousePos.x;
        float my = (float)mousePos.y;

        // Update button hover states
        for (auto& btn : buttons) btn.update(mx, my);
        for (auto& btn : controlButtons) btn.update(mx, my);
        
        // Update visualizer auto-play
        visualizer.update();

        // Event handling
        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
            
            // Mouse button pressed
            if (const auto* mouseEvent = event->getIf<sf::Event::MouseButtonPressed>()) {
                float clickX = (float)mouseEvent->position.x;
                float clickY = (float)mouseEvent->position.y;
                
                if (mouseEvent->button == sf::Mouse::Button::Left) {
                    // Check menu buttons
                    bool clickedMenu = false;
                    for (auto& btn : buttons) {
                        if (btn.isClicked(clickX, clickY)) {
                            if (btn.onClick) btn.onClick();
                            clickedMenu = true;
                            break;
                        }
                    }
                    
                    // Check control buttons
                    if (!clickedMenu) {
                        for (auto& btn : controlButtons) {
                            if (btn.isClicked(clickX, clickY)) {
                                if (btn.onClick) btn.onClick();
                                clickedMenu = true;
                                break;
                            }
                        }
                    }
                    
                    // Click in graph area
                    if (!clickedMenu && graphArea.contains({clickX, clickY})) {
                        int clickedNode = graph.getNodeClicked(clickX, clickY);
                        
                        if (currentMode == AppMode::EDITOR) {
                            if (clickedNode == -1) {
                                // Add new node
                                graph.addNode(clickX, clickY);
                            } else {
                                // Start dragging
                                renderer.startDragging(clickedNode, clickX, clickY);
                                renderer.setSelectedNode(clickedNode);
                            }
                        }
                        else if (currentMode == AppMode::SELECT_START) {
                            if (clickedNode != -1) {
                                selectedStartNode = clickedNode;
                                renderer.setSelectedNode(clickedNode);
                                
                                // Algorithms that need only start node
                                if (pendingAlgo == PendingAlgorithm::BFS) {
                                    visualizer.runBFS(graph, selectedStartNode);
                                    currentMode = AppMode::RUNNING_ALGORITHM;
                                }
                                else if (pendingAlgo == PendingAlgorithm::DFS) {
                                    visualizer.runDFS(graph, selectedStartNode);
                                    currentMode = AppMode::RUNNING_ALGORITHM;
                                }
                                else if (pendingAlgo == PendingAlgorithm::PRIM) {
                                    visualizer.runPrimMST(graph, selectedStartNode);
                                    currentMode = AppMode::RUNNING_ALGORITHM;
                                }
                                else if (pendingAlgo == PendingAlgorithm::FLEURY) {
                                    visualizer.runFleuryEulerian(graph, selectedStartNode);
                                    currentMode = AppMode::RUNNING_ALGORITHM;
                                }
                                else if (pendingAlgo == PendingAlgorithm::HIERHOLZER) {
                                    visualizer.runHierholzerEulerian(graph, selectedStartNode);
                                    currentMode = AppMode::RUNNING_ALGORITHM;
                                }
                                // Algorithms that need two nodes
                                else if (pendingAlgo == PendingAlgorithm::DIJKSTRA ||
                                         pendingAlgo == PendingAlgorithm::FORD_FULKERSON) {
                                    currentMode = AppMode::SELECT_END;
                                    std::cout << "Now click END/SINK node\n";
                                }
                            }
                        }
                        else if (currentMode == AppMode::SELECT_END) {
                            if (clickedNode != -1 && clickedNode != selectedStartNode) {
                                selectedEndNode = clickedNode;
                                renderer.setSecondSelectedNode(clickedNode);
                                
                                if (pendingAlgo == PendingAlgorithm::DIJKSTRA) {
                                    visualizer.runDijkstra(graph, selectedStartNode, selectedEndNode);
                                }
                                else if (pendingAlgo == PendingAlgorithm::FORD_FULKERSON) {
                                    visualizer.runFordFulkerson(graph, selectedStartNode, selectedEndNode);
                                }
                                currentMode = AppMode::RUNNING_ALGORITHM;
                            }
                        }
                        else if (currentMode == AppMode::VIEW_REPRESENTATION) {
                            currentMode = AppMode::EDITOR;
                            currentRepresentation = "";
                        }
                    }
                }
                
                // Right click - create edge or delete node
                if (mouseEvent->button == sf::Mouse::Button::Right && currentMode == AppMode::EDITOR) {
                    if (graphArea.contains({clickX, clickY})) {
                        int clickedNode = graph.getNodeClicked(clickX, clickY);
                        
                        if (clickedNode != -1) {
                            int prevSelected = renderer.getSelectedNode();
                            
                            if (prevSelected == -1) {
                                // First selection
                                renderer.setSelectedNode(clickedNode);
                            } else if (prevSelected == clickedNode) {
                                // Clicked same node - deselect
                                renderer.setSelectedNode(-1);
                            } else {
                                // Different node - prepare to create edge with weight
                                pendingEdgeFrom = prevSelected;
                                pendingEdgeTo = clickedNode;
                                weightInput = "";
                                currentMode = AppMode::INPUT_WEIGHT;
                                std::cout << "Enter weight for edge " << prevSelected << " -> " << clickedNode << " (then press Enter):\n";
                            }
                        } else {
                            renderer.setSelectedNode(-1);
                        }
                    }
                }
                
                // Middle click - delete node
                if (mouseEvent->button == sf::Mouse::Button::Middle && currentMode == AppMode::EDITOR) {
                    if (graphArea.contains({clickX, clickY})) {
                        int clickedNode = graph.getNodeClicked(clickX, clickY);
                        if (clickedNode != -1) {
                            graph.removeNode(clickedNode);
                            renderer.setSelectedNode(-1);
                        }
                    }
                }
            }
            
            // Mouse button released
            if (const auto* mouseEvent = event->getIf<sf::Event::MouseButtonReleased>()) {
                if (mouseEvent->button == sf::Mouse::Button::Left) {
                    renderer.stopDragging();
                }
            }
            
            // Mouse moved (for dragging)
            if (const auto* mouseEvent = event->getIf<sf::Event::MouseMoved>()) {
                if (renderer.isDragging()) {
                    renderer.updateDragging((float)mouseEvent->position.x, 
                                           (float)mouseEvent->position.y, graph);
                }
            }
            
            // Keyboard input
            if (const auto* keyEvent = event->getIf<sf::Event::KeyPressed>()) {
                // ESC to cancel selection or go back to editor
                if (keyEvent->code == sf::Keyboard::Key::Escape) {
                    if (currentMode != AppMode::EDITOR) {
                        visualizer.clear();
                        currentMode = AppMode::EDITOR;
                        renderer.setSelectedNode(-1);
                        renderer.setSecondSelectedNode(-1);
                        currentRepresentation = "";
                    }
                }
                
                // Delete key to remove selected node
                if (keyEvent->code == sf::Keyboard::Key::Delete || 
                    keyEvent->code == sf::Keyboard::Key::Backspace) {
                    int sel = renderer.getSelectedNode();
                    if (sel != -1 && currentMode == AppMode::EDITOR) {
                        graph.removeNode(sel);
                        renderer.setSelectedNode(-1);
                    }
                }
                
                // Space to advance step
                if (keyEvent->code == sf::Keyboard::Key::Space) {
                    if (currentMode == AppMode::RUNNING_ALGORITHM) {
                        visualizer.nextStep();
                    }
                }
                
                // Left/Right arrows for step navigation
                if (keyEvent->code == sf::Keyboard::Key::Right) {
                    visualizer.nextStep();
                }
                if (keyEvent->code == sf::Keyboard::Key::Left) {
                    visualizer.prevStep();
                }
                
                // Enter to confirm weight input
                if (keyEvent->code == sf::Keyboard::Key::Enter && currentMode == AppMode::INPUT_WEIGHT) {
                    double weight = 1.0;
                    if (!weightInput.empty()) {
                        try {
                            weight = std::stod(weightInput);
                        } catch (...) {
                            weight = 1.0;
                        }
                    }
                    graph.addEdge(pendingEdgeFrom, pendingEdgeTo, weight, graph.isDirectedGraph());
                    std::cout << "Created edge " << pendingEdgeFrom << " -> " << pendingEdgeTo << " with weight " << weight << "\n";
                    renderer.setSelectedNode(-1);
                    currentMode = AppMode::EDITOR;
                    weightInput = "";
                    pendingEdgeFrom = -1;
                    pendingEdgeTo = -1;
                }
                
                // Backspace in weight input mode
                if (keyEvent->code == sf::Keyboard::Key::Backspace && currentMode == AppMode::INPUT_WEIGHT) {
                    if (!weightInput.empty()) {
                        weightInput.pop_back();
                    }
                }
                
                // ESC to cancel weight input
                if (keyEvent->code == sf::Keyboard::Key::Escape && currentMode == AppMode::INPUT_WEIGHT) {
                    currentMode = AppMode::EDITOR;
                    weightInput = "";
                    renderer.setSelectedNode(-1);
                    pendingEdgeFrom = -1;
                    pendingEdgeTo = -1;
                }

                if (keyEvent->code == sf::Keyboard::Key::W && currentMode == AppMode::EDITOR) {
                    int sel = renderer.getSelectedNode();
                    if (sel != -1) {
                        std::cout << "Enter weight for edges from node " << sel << ": ";
                    }
                }
            }
            
            if (const auto* textEvent = event->getIf<sf::Event::TextEntered>()) {
                if (currentMode == AppMode::INPUT_WEIGHT) {
                    char c = static_cast<char>(textEvent->unicode);
                    if ((c >= '0' && c <= '9') || c == '.') {
                        weightInput += c;
                    }
                }
                else if (renderer.isEditingEdgeWeight()) {
                    if (textEvent->unicode == '\r' || textEvent->unicode == '\n') {
                        renderer.confirmEdgeWeight(graph);
                    } else if (textEvent->unicode < 128) {
                        renderer.updateEdgeWeightInput(static_cast<char>(textEvent->unicode), graph);
                    }
                }
            }
        }

        // ==================== RENDERING ====================
        window.clear(sf::Color(30, 30, 35));

        sf::RectangleShape menuPanel({MENU_WIDTH, WINDOW_HEIGHT});  // Use constant
        menuPanel.setFillColor(sf::Color(45, 45, 50));
        window.draw(menuPanel);
        
        window.draw(editLabel);
        window.draw(fileLabel);
        window.draw(basicLabel);
        window.draw(advLabel);
        window.draw(repLabel);
        
        for (auto& btn : buttons) btn.render(window);

        sf::RectangleShape bottomPanel({WINDOW_WIDTH - MENU_WIDTH, BOTTOM_PANEL_HEIGHT});
        bottomPanel.setPosition({MENU_WIDTH, WINDOW_HEIGHT - BOTTOM_PANEL_HEIGHT});  // Use constant
        bottomPanel.setFillColor(sf::Color(40, 40, 45));
        window.draw(bottomPanel);
        

        for (auto& btn : controlButtons) btn.render(window);
        

        renderer.render(graph, visualizer.isRunning() ? &visualizer : nullptr);

        sf::Text statusText(font, "", 16);
        statusText.setPosition({MENU_WIDTH + 10, 10});
        statusText.setFillColor(sf::Color::White);
        
        std::string modeStr = "";
        switch (currentMode) {
            case AppMode::EDITOR:
                modeStr = "EDITOR MODE | Left: Add Node | Right: Select/Connect | Middle: Delete | Drag to move";
                break;
            case AppMode::SELECT_START:
                modeStr = "SELECT START NODE...";
                break;
            case AppMode::SELECT_END:
                modeStr = "SELECT END NODE...";
                break;
            case AppMode::RUNNING_ALGORITHM:
                modeStr = "ALGORITHM RUNNING | Space/Right: Next | Left: Prev | ESC: Stop";
                break;
            case AppMode::VIEW_REPRESENTATION:
                modeStr = "VIEWING REPRESENTATION | Click to close";
                break;
            case AppMode::INPUT_WEIGHT:
                modeStr = "ENTER WEIGHT: " + (weightInput.empty() ? "_" : weightInput) + " | Press ENTER to confirm | ESC to cancel";
                break;
        }
        statusText.setString(modeStr);
        window.draw(statusText);
        
        sf::Text infoText(font, "", 14);
        infoText.setPosition({MENU_WIDTH + 10, 35});
        infoText.setFillColor(sf::Color(180, 180, 180));
        std::string infoStr = "Nodes: " + std::to_string(graph.getNodeCount()) + 
                             " | Edges: " + std::to_string(graph.getEdgeCount()) +
                             " | " + (graph.isDirectedGraph() ? "Directed" : "Undirected");
        infoText.setString(infoStr);
        window.draw(infoText);
        
        if (visualizer.isRunning()) {
            sf::Text algoText(font, "", 16);
            algoText.setPosition({MENU_WIDTH + 10, WINDOW_HEIGHT - BOTTOM_PANEL_HEIGHT + 50});  // Use constant
            algoText.setFillColor(sf::Color::Yellow);
            
            std::string stepInfo = "Step " + std::to_string(visualizer.getCurrentStepIndex() + 1) + 
                                  "/" + std::to_string(visualizer.getTotalSteps());
            algoText.setString(stepInfo);
            window.draw(algoText);
            
            sf::Text descText(font, "", 14);
            descText.setPosition({MENU_WIDTH + 10, WINDOW_HEIGHT - BOTTOM_PANEL_HEIGHT + 75});  // Use constant
            descText.setFillColor(sf::Color::White);
            descText.setString(visualizer.getCurrentDescription());
            window.draw(descText);
        }
        
        if (currentMode == AppMode::VIEW_REPRESENTATION && !currentRepresentation.empty()) {
            sf::RectangleShape repBg({600, 400});
            repBg.setPosition({MENU_WIDTH + 100, 100});
            repBg.setFillColor(sf::Color(20, 20, 25, 240));
            repBg.setOutlineColor(sf::Color::White);
            repBg.setOutlineThickness(2);
            window.draw(repBg);
            
            sf::Text repTitle(font, "Graph Representation", 20);
            repTitle.setPosition({MENU_WIDTH + 120, 110});
            repTitle.setFillColor(sf::Color::Cyan);
            window.draw(repTitle);
            
            sf::Text repContent(font, currentRepresentation, 14);
            repContent.setPosition({MENU_WIDTH + 120, 145});
            repContent.setFillColor(sf::Color::White);
            window.draw(repContent);
            
            sf::Text closeHint(font, "Click anywhere to close", 12);
            closeHint.setPosition({MENU_WIDTH + 120, 480});
            closeHint.setFillColor(sf::Color(150, 150, 150));
            window.draw(closeHint);
        }
        
        if (currentMode == AppMode::INPUT_WEIGHT) {
            // Dialog box position and size
            float dialogX = MENU_WIDTH + 300;
            float dialogY = 350;
            float dialogW = 320;
            float dialogH = 120;
            float padding = 15;
            
            sf::RectangleShape inputBg({dialogW, dialogH});
            inputBg.setPosition({dialogX, dialogY});
            inputBg.setFillColor(sf::Color(30, 30, 35, 250));
            inputBg.setOutlineColor(sf::Color::Yellow);
            inputBg.setOutlineThickness(2);
            window.draw(inputBg);
            
            sf::Text inputTitle(font, "Enter Edge Weight", 18);
            inputTitle.setPosition({dialogX + padding, dialogY + padding});
            inputTitle.setFillColor(sf::Color::Yellow);
            window.draw(inputTitle);
            
            sf::Text inputLabel(font, "Edge: " + std::to_string(pendingEdgeFrom) + " -> " + std::to_string(pendingEdgeTo), 14);
            inputLabel.setPosition({dialogX + padding, dialogY + padding + 30});
            inputLabel.setFillColor(sf::Color::White);
            window.draw(inputLabel);
            
            sf::RectangleShape inputBox({150, 30});
            inputBox.setPosition({dialogX + padding, dialogY + padding + 55});
            inputBox.setFillColor(sf::Color::White);
            inputBox.setOutlineColor(sf::Color::Cyan);
            inputBox.setOutlineThickness(1);
            window.draw(inputBox);
            
            sf::Text inputText(font, weightInput.empty() ? "1" : weightInput, 16);
            inputText.setPosition({dialogX + padding + 10, dialogY + padding + 58});
            inputText.setFillColor(sf::Color::Black);
            window.draw(inputText);
            
            sf::Text inputHint(font, "Press ENTER to confirm", 12);
            inputHint.setPosition({dialogX + padding + 160, dialogY + padding + 62});
            inputHint.setFillColor(sf::Color(150, 150, 150));
            window.draw(inputHint);
        }
        
        sf::Text helpText(font, "ESC: Back to Editor | SPACE: Next Step | Arrows: Navigate Steps", 12);
        helpText.setPosition({MENU_WIDTH + 600, WINDOW_HEIGHT - BOTTOM_PANEL_HEIGHT + 10});  // Use constant
        helpText.setFillColor(sf::Color(120, 120, 120));
        window.draw(helpText);

        window.display();
    }
    
    return 0;
}
