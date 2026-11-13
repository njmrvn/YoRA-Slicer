@echo off
rem YoRA Slicer - builds the program folder %USERPROFILE%\.yora\dist\YoRA Slicer, with YoRA Slicer.exe inside.
rem The tools (PyInstaller) go in their own environment and only this PC needs them; the program runs without Python.
setlocal
cd /d "%~dp0"
set "APP_PY=%USERPROFILE%\.yora\venv\Scripts\python.exe"
set "BUILD=%USERPROFILE%\.yora\build"
set "BUILD_PY=%BUILD%\venv\Scripts\python.exe"
if not exist "%APP_PY%" (
  call "%~dp0Setup YoRA.bat" || exit /b 1
)
if not exist "%BUILD_PY%" (
  echo Creating the build environment ...
  "%APP_PY%" -m venv "%BUILD%\venv" || goto failed
)
echo Installing the build tools ...
"%BUILD_PY%" -m pip install --disable-pip-version-check -q -r requirements.txt pyinstaller || goto failed
echo Building, this takes a few minutes ...
"%BUILD_PY%" -m PyInstaller --noconfirm --clean --windowed --onedir --name "YoRA Slicer" ^
  --distpath "%USERPROFILE%\.yora\dist" --workpath "%BUILD%\work" --specpath "%BUILD%" ^
  --collect-submodules yora --exclude-module tkinter ^
  --add-data "%~dp0web;web" --add-data "%~dp0samples;samples" --add-data "%~dp0posts\*.py;posts" --add-data "%~dp0LICENSE;." ^
  yora_slicer.py || goto failed
echo.
echo Done: %USERPROFILE%\.yora\dist\YoRA Slicer\YoRA Slicer.exe
exit /b 0

:failed
echo The build failed, see the messages above.
exit /b 1
