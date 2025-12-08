CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -pedantic
INCLUDES := -I./include

SRC_DIR := src
SRCS := $(SRC_DIR)/main.cpp \
        $(SRC_DIR)/trace_reader.cpp \
        $(SRC_DIR)/msi_cache.cpp
OBJS := $(SRCS:.cpp=.o)

TARGET ?= baseline
TRACE ?= dataset/data/MT0-canneal

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@

$(SRC_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

run: $(TARGET)
	./$(TARGET) $(TRACE)

clean:
	rm -f $(OBJS) $(TARGET)
