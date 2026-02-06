# Cross-platform Makefile for Unit Tests

# Detect OS
ifeq ($(OS),Windows_NT)
    # Windows
    SHELL = cmd.exe
    CXX = C:/msys64/ucrt64/bin/clang++.exe
    RM = del /Q
    EXT = .exe
    
    # Run recipe for Windows (chains commands with &&)
    # Note: We run all tests sequentially
    RUN_CMD = set PATH=C:\msys64\ucrt64\bin;%PATH% && test_gait.exe && test_ik.exe && test_kinematics.exe && test_integration.exe
else
    # Mac / Linux
    CXX = clang++
    RM = rm -f
    EXT = 
    
    # Run recipe for Unix
    RUN_CMD = ./test_gait && ./test_ik && ./test_kinematics && ./test_integration
endif

CXXFLAGS = -std=c++17 -Wall -Wextra -I../lib/HexapodCore/include

# Targets
TARGET_GAIT = test_gait$(EXT)
TARGET_IK = test_ik$(EXT)
TARGET_KIN = test_kinematics$(EXT)
TARGET_INT = test_integration$(EXT)

SOURCES_GAIT = test_gait_math.cpp ../lib/HexapodCore/src/GaitMath.cpp
SOURCES_IK = test_leg_ik.cpp ../lib/HexapodCore/src/LegIK.cpp
SOURCES_KIN = test_kinematics.cpp ../lib/HexapodCore/src/Hexapod.cpp ../lib/HexapodCore/src/LegIK.cpp ../lib/HexapodCore/src/GaitMath.cpp
SOURCES_INT = test_hexapod_integration.cpp ../lib/HexapodCore/src/Hexapod.cpp ../lib/HexapodCore/src/LegIK.cpp ../lib/HexapodCore/src/GaitMath.cpp

all: $(TARGET_GAIT) $(TARGET_IK) $(TARGET_KIN) $(TARGET_INT) run

$(TARGET_GAIT): $(SOURCES_GAIT)
	$(CXX) $(CXXFLAGS) $(SOURCES_GAIT) -o $(TARGET_GAIT)

$(TARGET_IK): $(SOURCES_IK)
	$(CXX) $(CXXFLAGS) $(SOURCES_IK) -o $(TARGET_IK)

$(TARGET_KIN): $(SOURCES_KIN)
	$(CXX) $(CXXFLAGS) $(SOURCES_KIN) -o $(TARGET_KIN)

$(TARGET_INT): $(SOURCES_INT)
	$(CXX) $(CXXFLAGS) $(SOURCES_INT) -o $(TARGET_INT)

run: $(TARGET_GAIT) $(TARGET_IK) $(TARGET_INT)
	$(RUN_CMD)

clean:
	$(RM) $(TARGET_GAIT) $(TARGET_IK) $(TARGET_KIN) $(TARGET_INT) run_tests.exe
