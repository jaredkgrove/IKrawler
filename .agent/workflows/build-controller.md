---
description: Build the Webots hexapod controller
---

# Build Webots Controller

Rebuilds the hexapod_controller for Webots simulation.

## Prerequisites
- Stop the simulation in Webots first (the .exe file must not be in use)

## Steps

1. Open a terminal in VS Code (Ctrl+`)

2. Run the build command:
// turbo
```powershell
cd c:\jaredcode\jareds-hexapod\webots\controllers\hexapod_controller; C:/msys64/ucrt64/bin/mingw32-make.exe
```

3. Reload the world in Webots (Ctrl+Shift+R)

## Full Rebuild (Clean Build)

If you need to rebuild everything from scratch:
```powershell
cd c:\jaredcode\jareds-hexapod\webots\controllers\hexapod_controller; Remove-Item *.o -ErrorAction SilentlyContinue; Remove-Item hexapod_controller.exe -ErrorAction SilentlyContinue; C:/msys64/ucrt64/bin/mingw32-make.exe
```

## Troubleshooting

- **Permission denied**: The simulation is still running. Stop it in Webots first.
- **make not found**: Use the full path `C:/msys64/ucrt64/bin/mingw32-make.exe`
