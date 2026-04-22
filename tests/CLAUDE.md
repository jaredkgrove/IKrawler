# Unit Tests

## Running Tests

```powershell
cd c:\jaredcode\jareds-hexapod\tests; .\make
```

Expected output:
```
Running Hexapod Unit Tests...
ALL GAIT MATH TESTS PASSED
Running LegIK Unit Tests...
ALL LEG IK TESTS PASSED
--------------------------------
ALL HEXAPOD INTEGRATION TESTS PASSED
```

## Scope

Tests cover gait math, inverse kinematics, and controller integration. They run as pure C++ on the host (no ESP32 or Webots needed). Test the public API — do not mock HexapodCore internals.
