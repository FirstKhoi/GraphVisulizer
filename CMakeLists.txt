cmake_minimum_required(VERSION 3.16)
project(GraphVisualization VERSION 1.0.0 LANGUAGES CXX)

# ===== macOS:
if(APPLE)
    set(CMAKE_OSX_DEPLOYMENT_TARGET "13.0" CACHE STRING "Minimum macOS version")
endif()

# ===== C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ===== Output directories =====
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin)

# ===== Find SFML =====
# Try SFML 2.5+ first (uses modern CMake targets)
find_package(SFML 2.5 COMPONENTS graphics window system QUIET)

if(NOT SFML_FOUND)
    # Try SFML 3.0 if 2.5 not found
    find_package(SFML 3.0 COMPONENTS Graphics Window System QUIET)
endif()

if(NOT SFML_FOUND)
    message(FATAL_ERROR "SFML not found! Please install SFML:\n"
        "  macOS: brew install sfml\n"
        "  Ubuntu: sudo apt install libsfml-dev\n"
        "  Windows: Download from https://www.sfml-dev.org/")
endif()

message(STATUS "Found SFML ${SFML_VERSION}")

# ===== Source files =====
set(HEADERS
    include/Graph.h
    include/GraphRenderer.h
    include/Button.h
    include/AlgorithmVisualizer.h
    include/FileIO.h
)

set(SOURCES
    main.cpp
    src/Graph.cpp
    src/GraphRenderer.cpp
    src/Button.cpp
    src/AlgorithmVisualizer.cpp
    src/FileIO.cpp
)

# ===== Create executable =====
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# ===== Include directories =====
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include)

# ===== Link SFML =====
if(SFML_VERSION VERSION_GREATER_EQUAL "3.0")
    target_link_libraries(${PROJECT_NAME} PRIVATE SFML::Graphics SFML::Window SFML::System)
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE sfml-graphics sfml-window sfml-system)
endif()

# ===== Platform specific settings =====
if(WIN32)
    # Windows: Copy DLLs
    if(SFML_VERSION VERSION_GREATER_EQUAL "3.0")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:SFML::Graphics>
                $<TARGET_FILE:SFML::Window>
                $<TARGET_FILE:SFML::System>
                $<TARGET_FILE_DIR:${PROJECT_NAME}>
        )
    endif()
elseif(APPLE)
    # macOS: Set RPATH
    set_target_properties(${PROJECT_NAME} PROPERTIES
        MACOSX_BUNDLE FALSE
        INSTALL_RPATH "@executable_path/../Frameworks"
        BUILD_WITH_INSTALL_RPATH TRUE
    )
endif()

# ===== Download DejaVuSans font if not exists =====
set(FONT_DIR "${CMAKE_SOURCE_DIR}/resources/fonts")
set(FONT_FILE "${FONT_DIR}/DejaVuSans.ttf")

if(NOT EXISTS "${FONT_FILE}")
    message(STATUS "DejaVuSans.ttf not found, downloading...")
    
    # Create fonts directory
    file(MAKE_DIRECTORY "${FONT_DIR}")
    
    # Download font from GitHub
    file(DOWNLOAD
        "https://github.com/dejavu-fonts/dejavu-fonts/raw/master/ttf/DejaVuSans.ttf"
        "${FONT_FILE}"
        SHOW_PROGRESS
        STATUS DOWNLOAD_STATUS
    )
    
    list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
    if(STATUS_CODE EQUAL 0)
        message(STATUS "DejaVuSans.ttf downloaded successfully")
    else()
        message(WARNING "Failed to download DejaVuSans.ttf. Please download manually from:\n"
            "https://github.com/dejavu-fonts/dejavu-fonts/releases")
    endif()
else()
    message(STATUS "Found DejaVuSans.ttf")
endif()

# ===== Copy resources =====
if(EXISTS "${CMAKE_SOURCE_DIR}/resources")
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/resources"
            "$<TARGET_FILE_DIR:${PROJECT_NAME}>/resources"
        COMMENT "Copying resources..."
    )
else()
    message(STATUS "No resources directory found, skipping resource copy")
endif()
