@echo off
rem YoRA Slicer - runs on its own Python environment (see "Setup YoRA.bat").
cd /d "%~dp0"
set "VENV_PY=%USERPROFILE%\.yora\venv\Scripts\python.exe"
if not exist "%VENV_PY%" (
  echo First start: setting up Python packages, this takes a minute ...
  call "%~dp0Setup YoRA.bat" || exit /b 1
)
"%VENV_PY%" -m yora.server --open %*
