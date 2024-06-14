# Compiler and compiler flags
CXX = g++
CXXFLAGS = -std=c++17 -g -Wall -Wextra -pedantic -I /usr/include/eigen3
SFML_INCLUDE = -I/usr/include/SFML/include  # Adjust for SFML headers

# SFML library paths and flags
SFML_LIBS = -L/usr/lib/x86_64-linux-gnu \
            -lsfml-graphics \
            -lsfml-window \
            -lsfml-system \
            -lsfml-audio \
            -lsfml-network

# Source files and object files
SRCS = main.cpp DataLoader.cpp Dataset.cpp Observation.cpp ConBear.cpp Visualization.cpp
OBJS = $(SRCS:.cpp=.o)
BUILD_DIR = build
BUILD_OBJS = $(addprefix $(BUILD_DIR)/, $(OBJS))

# Target executable
TARGET = $(BUILD_DIR)/slam_ekf_algorithm

# Phony targets
.PHONY: all clean

# Default target
all: $(BUILD_DIR) $(TARGET)

# Create the build directory if it doesn't exist
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Linking the target executable
$(TARGET): $(BUILD_OBJS)
	$(CXX) $(CXXFLAGS) $(SFML_INCLUDE) -o $@ $^ $(SFML_LIBS)

# Compiling source files into object files in the build directory
$(BUILD_DIR)/%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(SFML_INCLUDE) -c $< -o $@

# Cleaning up build artifacts
clean:
	rm -f $(BUILD_OBJS) $(TARGET)
