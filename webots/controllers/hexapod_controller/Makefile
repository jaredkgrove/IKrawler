# Webots Makefile for hexapod_controller
CXX = C:/msys64/ucrt64/bin/clang++.exe
CXXFLAGS = -std=c++17 -Wall -Wextra -MMD -MP

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
          ../../../lib/HexapodCore/src/LegIK.cpp \
          ../../../lib/HexapodCore/src/GaitMath.cpp

# Object files and dependency files
OBJECTS = $(SOURCES:.cpp=.o)
DEPS = $(SOURCES:.cpp=.d)

# Target executable
TARGET = hexapod_controller.exe

# Build rules
all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBRARIES)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $< -o $@

# Include generated dependency files (if they exist)
-include $(DEPS)

clean:
	rm -f $(OBJECTS) $(DEPS) $(TARGET)

.PHONY: all clean
