
# Compiler and compiler flags
CXX = g++
CXXFLAGS = -std=c++17 -g -Wall -Wextra -pedantic
CXXFLAGS = -I /usr/include/eigen3 -std=c++11
# Source files and object files
SRCS = main.cpp DataLoader.cpp Dataset.cpp #Observation.cpp EKFAlgorithm.cpp Visualization.cpp
OBJS = $(SRCS:.cpp=.o)
BUILD_DIR = build
BUILD_OBJS = $(addprefix $(BUILD_DIR)/, $(OBJS))

# Target executable
TARGET = $(BUILD_DIR)/myprogram

# Phony targets
.PHONY: all clean

# Default target
all: $(BUILD_DIR) $(TARGET)

# Create the build directory if it doesn't exist
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Linking the target executable
$(TARGET): $(BUILD_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compiling source files into object files in the build directory
$(BUILD_DIR)/%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Cleaning up build artifacts
clean:
	rm -f $(BUILD_OBJS) $(TARGET)
