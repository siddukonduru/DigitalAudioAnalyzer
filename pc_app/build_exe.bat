@echo off
cd /d "%~dp0"
REM Builds HU_Audio_Analyzer.exe (one-file) - run from this folder on Windows with Python 3.10+ installed
py -m pip install --upgrade pip
py -m pip install -r requirements.txt
py -m PyInstaller --noconfirm --onefile --windowed --name HU_Audio_Analyzer app.py
echo.
echo Done: dist\HU_Audio_Analyzer.exe
pause
