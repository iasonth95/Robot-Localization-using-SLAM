cmake_minimum_required(VERSION 3.10)
project(RobotLocalizationSLAM VERSION 0.1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -Wextra -pedantic")

# Find and include Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Find and include SFML
find_package(SFML 2.5 COMPONENTS graphics window system audio network REQUIRED)
include_directories(${SFML_INCLUDE_DIRS})

# Source files
set(SOURCES
    main.cpp
    DataLoader.cpp
    Dataset.cpp
    Observation.cpp
    ConBear.cpp
    Visualization.cpp
)

# Add executable
add_executable(slam_ekf_algorithm ${SOURCES})

# Link libraries
target_link_libraries(slam_ekf_algorithm Eigen3::Eigen sfml-graphics sfml-window sfml-system sfml-audio sfml-network)

# Set output directory
set_target_properties(slam_ekf_algorithm PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build)
