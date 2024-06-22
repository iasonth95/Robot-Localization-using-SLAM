CXX = g++
CXXFLAGS = -std=c++17 -g -Wall -Wextra -pedantic -I /usr/include/eigen3
SFML_INCLUDE = -I/usr/include/SFML/include

SFML_LIBS = -L/usr/lib/x86_64-linux-gnu \
            -lsfml-graphics \
            -lsfml-window \
            -lsfml-system \
            -lsfml-audio \
            -lsfml-network

SRCS = main.cpp DataLoader.cpp Dataset.cpp Observation.cpp ConBear.cpp Visualization.cpp
OBJS = $(SRCS:.cpp=.o)
BUILD_DIR = build
BUILD_OBJS = $(addprefix $(BUILD_DIR)/, $(OBJS))

TARGET = $(BUILD_DIR)/slam_ekf_algorithm

.PHONY: all clean

all: $(BUILD_DIR) $(TARGET)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TARGET): $(BUILD_OBJS)
	$(CXX) $(CXXFLAGS) $(SFML_INCLUDE) -o $@ $^ $(SFML_LIBS)

$(BUILD_DIR)/%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(SFML_INCLUDE) -c $< -o $@

clean:
	rm -f $(BUILD_OBJS) $(TARGET)
