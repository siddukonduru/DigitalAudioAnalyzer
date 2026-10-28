@echo off
cd /d "%~dp0"
set PY=
where py >nul 2>&1 && set PY=py
if not defined PY ( where python >nul 2>&1 && set PY=python )
if not defined PY (
  echo.
  echo Python is not installed on this PC.
  echo Install Python 3.12 from https://www.python.org/downloads/windows/
  echo and tick "Add python.exe to PATH" on the first installer screen, then run this file again.
  echo.
  pause
  exit /b 1
)
echo Using: %PY%
%PY% --version
echo Installing / checking packages (first time takes a few minutes)...
%PY% -m pip install -r requirements.txt
if errorlevel 1 ( echo. & echo pip install failed - see the messages above. & pause & exit /b 1 )
echo Starting HU Audio Analyzer...
%PY% app.py
if errorlevel 1 ( echo. & echo The app exited with an error - see the messages above. )
pause
