@echo off
rem YoRA Slicer - one-time setup: a private Python environment with numpy and OpenCV.
rem   Setup YoRA.bat           create or update the environment
rem The environment lives outside OneDrive, in %USERPROFILE%\.yora\venv, so it is not synced.
setlocal
set "VENV=%USERPROFILE%\.yora\venv"
set "PY="
call :try py
call :try python
call :try python3
if not defined PY goto nopython
if exist "%VENV%\Scripts\python.exe" goto install
echo Creating the Python environment in %VENV% ...
%PY% -m venv "%VENV%"
if errorlevel 1 goto failed
:install
echo Installing packages: numpy, OpenCV ...
"%VENV%\Scripts\python.exe" -m pip install --disable-pip-version-check -q -r "%~dp0requirements.txt"
if errorlevel 1 goto failed
:done
echo Setup done.
exit /b 0

:try
if defined PY exit /b 0
%1 -c "import sys; sys.exit(sys.version_info < (3, 10))" >nul 2>&1
if not errorlevel 1 set "PY=%1"
exit /b 0

:nopython
echo Python 3.10 or newer was not found.
echo Install it from https://www.python.org/downloads/ or the Microsoft Store, search "Python 3.12", then run this again.
pause
exit /b 1

:failed
echo Setup failed, see the messages above.
pause
exit /b 1
