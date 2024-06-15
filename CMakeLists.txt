cmake_minimum_required(VERSION 3.10)
project(RL-SLAM)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always")
# Find Eigen3
find_package(Eigen3 3.3 REQUIRED)

# Find Armadillo
find_package(Armadillo REQUIRED)
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)
file(GLOB_RECURSE SRC_FILES src/*.cpp)
# Include directories for Eigen3 and Armadillo
include_directories(${EIGEN3_INCLUDE_DIR} ${ARMADILLO_INCLUDE_DIRS} ${SFML_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/include)
# Add your executable
add_executable(RL-SLAM ${SRC_FILES})

# Link Armadillo (Eigen is header-only, so no linking required)
target_link_libraries(RL-SLAM ${ARMADILLO_LIBRARIES} sfml-graphics sfml-window sfml-system)
