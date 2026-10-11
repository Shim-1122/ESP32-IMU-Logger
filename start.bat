@echo off
rem Double-click this file to set up and run the project on Windows.
cd /d "%~dp0"
where py >nul 2>nul
if %errorlevel%==0 (
    py -3 start.py %*
) else (
    python start.py %*
)
if %errorlevel%==9009 (
    echo.
    echo Python was not found. Install it from https://www.python.org/downloads/
    echo and tick "Add python.exe to PATH" on the first screen of the installer.
)
pause
