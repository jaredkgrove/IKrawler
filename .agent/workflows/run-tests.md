---
description: Run unit tests for the hexapod project
---

# Run Unit Tests

Compiles and runs the pure C++ unit tests to verify gait logic, inverse kinematics, and controller integration.

## Steps

1. Open a terminal in VS Code (Ctrl+`)

2. Go to the tests directory and run the make script:
// turbo
```powershell
cd c:\jaredcode\jareds-hexapod\tests; .\make
```

3. Verify all tests passed. Expected output:
```text
Running Hexapod Unit Tests...
ALL GAIT MATH TESTS PASSED
Running LegIK Unit Tests...
ALL LEG IK TESTS PASSED
--------------------------------
ALL HEXAPOD INTEGRATION TESTS PASSED
```
