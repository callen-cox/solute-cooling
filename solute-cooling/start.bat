@echo off
title Autonomous Thermal Controller
cd /d "%~dp0"
echo.
echo   Autonomous Thermal Controller
echo   -----------------------------
set "PY="
python --version >nul 2>nul && set "PY=python"
if not defined PY py --version >nul 2>nul && set "PY=py"
if not defined PY (
  echo   Python is not installed on this computer.
  echo   Get it from https://www.python.org/downloads/ and tick "Add python.exe to PATH",
  echo   then double-click start.bat again.
  echo.
  pause
  exit /b 1
)
echo   Using:
%PY% --version
echo   Installing packages ^(the first run takes a minute^)...
%PY% -m pip install --disable-pip-version-check -q -r requirements.txt
if errorlevel 1 (
  echo   Package install failed. Check the internet connection and try again.
  pause
  exit /b 1
)
echo.
echo   Starting. The dashboard opens at http://localhost:8000
echo   Keep this window open while you demo. Close it to stop.
echo.
%PY% -m coolant.server --open
pause
