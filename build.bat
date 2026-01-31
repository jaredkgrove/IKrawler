@echo off
REM Build the Webots hexapod controller
echo Building hexapod controller...
cd /d c:\jaredcode\jareds-hexapod\webots\controllers\hexapod_controller
C:\msys64\ucrt64\bin\mingw32-make.exe
if %ERRORLEVEL% EQU 0 (
    echo Build successful! Reload the world in Webots.
) else (
    echo Build failed! Make sure Webots simulation is stopped.
)
