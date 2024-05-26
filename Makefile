# Compiler and compiler flags
CXX = g++
CXXFLAGS = -std=c++17 -g -Wall -Wextra -pedantic 

# Source files and object files

SRCS = main.cpp DataLoader.cpp Dataset.cpp #Observation.cpp EKFAlgorithm.cpp Visualization.cpp
OBJS = $(SRCS:.cpp=.o)

# Target executable
TARGET = myprogram

# Phony targets
.PHONY: all clean

# Default target
all: $(TARGET)

# Linking the target executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compiling source files into object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Cleaning up build artifacts
clean:
	rm -f $(OBJS) $(TARGET)
