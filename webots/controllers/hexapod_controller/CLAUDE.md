# Webots Hexapod Controller

## Building

Stop the simulation in Webots first — the .exe must not be in use.

```powershell
cd c:\jaredcode\jareds-hexapod\webots\controllers\hexapod_controller; C:/msys64/ucrt64/bin/mingw32-make.exe
```

After building, reload the world in Webots (Ctrl+Shift+R).

### Clean Build

```powershell
cd c:\jaredcode\jareds-hexapod\webots\controllers\hexapod_controller; Remove-Item *.o -ErrorAction SilentlyContinue; Remove-Item hexapod_controller.exe -ErrorAction SilentlyContinue; C:/msys64/ucrt64/bin/mingw32-make.exe
```

## Troubleshooting

- **Permission denied**: The simulation is still running. Stop it in Webots first.
- **make not found**: Use the full path `C:/msys64/ucrt64/bin/mingw32-make.exe`.
