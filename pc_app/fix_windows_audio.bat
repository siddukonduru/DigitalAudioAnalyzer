@echo off
REM Restarts the Windows audio services (fixes "no sound / speaker flyout blank / analyzer can't open").
net session >nul 2>&1
if errorlevel 1 (
  echo Asking for administrator permission...
  powershell -NoProfile -Command "Start-Process -FilePath '%~f0' -Verb RunAs"
  exit /b
)
echo Restarting Windows Audio services...
powershell -NoProfile -Command "Restart-Service -Name AudioEndpointBuilder -Force -ErrorAction Continue; Start-Service Audiosrv; Get-Service AudioEndpointBuilder,Audiosrv | Format-Table Name,Status"
echo.
echo If both show Running, check the speaker icon, then run run.bat.
echo If a service is stuck "Stopping", reboot the laptop instead.
pause
