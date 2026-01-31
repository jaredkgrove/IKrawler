# Webots Makefile for hexapod_controller
CXX = C:/msys64/ucrt64/bin/clang++.exe
CXXFLAGS = -std=c++17 -Wall -Wextra

# Webots home directory (use environment variable if set, otherwise default)
WEBOTS_HOME ?= C:/Program Files/Webots

# Include directories
INCLUDE = -I"$(WEBOTS_HOME)/include/controller/cpp" \
          -I../../../lib/HexapodCore/include \
          -I../../../lib/HexapodCore/src

# Library directories and libraries
LIBRARIES = -L"$(WEBOTS_HOME)/lib/controller" -lCppController -lController

# Source files
SOURCES = hexapod_controller.cpp \
          WebotsServoImpl.cpp \
          ../../../lib/HexapodCore/src/Hexapod.cpp \
          ../../../lib/HexapodCore/src/LegIK.cpp

# Object files
OBJECTS = $(SOURCES:.cpp=.o)

# Target executable
TARGET = hexapod_controller.exe

# Build rules
all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBRARIES)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET)

.PHONY: all clean
