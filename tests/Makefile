# Cross-platform Makefile for Unit Tests

# Detect OS
ifeq ($(OS),Windows_NT)
    # Windows
    SHELL = cmd.exe
    CXX = C:/msys64/ucrt64/bin/clang++.exe
    RM = del /Q
    EXT = .exe
    
    # Run recipe for Windows (chains commands with &&)
    RUN_CMD = set PATH=C:\msys64\ucrt64\bin;%PATH% && run_tests.exe
else
    # Mac / Linux
    CXX = clang++
    RM = rm -f
    EXT = 
    
    # Run recipe for Unix
    RUN_CMD = ./run_tests
endif

CXXFLAGS = -std=c++17 -Wall -Wextra -I../lib/HexapodCore/include

TARGET = run_tests$(EXT)
SOURCES = test_gait_math.cpp ../lib/HexapodCore/src/GaitMath.cpp

all: $(TARGET) run

$(TARGET): $(SOURCES)
	$(CXX) $(CXXFLAGS) $(SOURCES) -o $(TARGET)

run: $(TARGET)
	$(RUN_CMD)

clean:
	$(RM) $(TARGET)
