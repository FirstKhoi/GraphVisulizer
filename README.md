# Graph Visualization Tool - DSA Learning Application

A comprehensive desktop application for creating, visualizing, and analyzing graphs for teaching and learning Data Structures and Algorithms.

## Features

### Core Features
- **Visual Graph Drawing**: Create and edit graphs interactively
  - Add/remove vertices (left-click to add, middle-click to delete)
  - Add/remove edges (right-click to connect nodes)
  - Support for directed and undirected graphs
  - Optional edge weights
  - Drag nodes to reposition

- **Save and Load Graphs**: Custom file format for persistence
  - Save current graph to file
  - Load previously saved graphs

- **Graph Representations**: View and convert between:
  - Adjacency Matrix
  - Adjacency List
  - Edge List

### Basic Algorithms (with step-by-step visualization)
- **BFS Traversal**: Breadth-First Search from selected vertex
- **DFS Traversal**: Depth-First Search from selected vertex
- **Dijkstra's Algorithm**: Find shortest path between two vertices
- **Bipartite Check**: Determine if graph is bipartite with 2-coloring

### Advanced Algorithms (with step-by-step visualization)
- **Prim's Algorithm**: Minimum Spanning Tree
- **Kruskal's Algorithm**: Minimum Spanning Tree
- **Ford-Fulkerson Algorithm**: Maximum Flow between source and sink
- **Fleury's Algorithm**: Finding Eulerian path/circuit
- **Hierholzer's Algorithm**: Finding Eulerian path/circuit

## Controls

### Mouse Controls
- **Left Click (empty area)**: Add new node
- **Left Click (on node)**: Select node for dragging
- **Right Click (on node)**: Select node for edge creation
- **Right Click (on second node)**: Create edge between selected nodes
- **Middle Click (on node)**: Delete node
- **Drag**: Move selected node

### Keyboard Controls
- **ESC**: Return to editor mode / Cancel operation
- **Space** or **Right Arrow**: Next algorithm step
- **Left Arrow**: Previous algorithm step
- **Delete/Backspace**: Delete selected node

## Building the Project

### Requirements
- CMake 3.15+
- C++17 compatible compiler
- SFML 3.0

### Build Instructions

```bash
# Navigate to project directory
cd GraphProject

# Create and enter build directory
mkdir build && cd build

# Generate build files
cmake ..

# Build the project
make -j4

# Run the application
./bin/GraphVisualization
```

### For Xcode (macOS)
```bash
cmake .. -G Xcode
open GraphVisualization.xcodeproj
```

## Project Structure

```
GraphProject/
├── CMakeLists.txt           # Build configuration
├── main.cpp                 # Main application entry point
├── include/
│   ├── Graph.h              # Graph data structure and algorithms
│   ├── GraphRenderer.h      # Visualization/rendering
│   ├── Button.h             # UI button component
│   ├── AlgorithmVisualizer.h # Algorithm step controller
│   └── FileIO.h             # File save/load utilities
├── src/
│   ├── Graph.cpp            # Graph implementation
│   ├── GraphRenderer.cpp    # Renderer implementation
│   ├── Button.cpp           # Button implementation
│   ├── AlgorithmVisualizer.cpp # Visualizer implementation
│   └── FileIO.cpp           # File I/O implementation
└── build/                   # Build output directory
```

## Architecture

The application follows a modular architecture:

1. **Data Layer** (`Graph.h/cpp`)
   - Graph data structure (nodes, edges)
   - All graph algorithms implementation
   - Graph representation conversions

2. **Visualization Layer** (`GraphRenderer.h/cpp`)
   - Rendering nodes and edges
   - Visual highlighting for algorithms
   - User interaction (dragging, selection)

3. **Algorithm Control** (`AlgorithmVisualizer.h/cpp`)
   - Step-by-step algorithm execution
   - Playback control (next, prev, auto-play)
   - State management for visualization

4. **File I/O** (`FileIO.h/cpp`)
   - Save/load graph to/from file
   - Export graph representations
   - Text format conversion

5. **UI Components** (`Button.h/cpp`)
   - Interactive buttons
   - Hover effects
   - Click handling

## File Format

Graphs are saved in a custom text format:
```
GRAPH_FILE_V1
DIRECTED 0
NODES 4
0 300 200
1 500 200
2 400 400
3 600 400
EDGES 4
0 1 1
1 2 2
2 3 3
0 3 4
```

## Known Limitations

- Edge weights must be positive integers for visualization
- Maximum recommended nodes: ~50 for optimal performance
- Font file (arial.ttf) must be in the same directory as executable

## Future Improvements

- Undo/Redo functionality
- More algorithm visualizations (A*, Bellman-Ford, etc.)
- Export to image/PDF
- Custom color themes
- Graph generation tools (random, complete, etc.)
- Multi-graph support

## Author

Luong Nhat Khoi - December 2025
