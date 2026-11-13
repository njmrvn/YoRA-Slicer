@echo off
rem YoRA Slicer - builds the installer %USERPROFILE%\.yora\dist\YoRA-Slicer-Setup-<version>.exe, one file to download.
rem It builds the program folder first (Build exe.bat), then packs it with Inno Setup 6 (installer\YoRA Slicer.iss).
rem Inno Setup is needed only on this PC:  winget install JRSoftware.InnoSetup
setlocal
cd /d "%~dp0"
set "ISCC=%LOCALAPPDATA%\Programs\Inno Setup 6\ISCC.exe"
if not exist "%ISCC%" set "ISCC=%ProgramFiles(x86)%\Inno Setup 6\ISCC.exe"
if not exist "%ISCC%" set "ISCC=%ProgramFiles%\Inno Setup 6\ISCC.exe"
if not exist "%ISCC%" goto no_inno
call "%~dp0Build exe.bat" || exit /b 1
set "VERSION="
for /f "tokens=2 delims='" %%v in ('findstr /b "__version__" yora\__init__.py') do set "VERSION=%%v"
if not defined VERSION set "VERSION=0.0.0"
echo Packing the installer, version %VERSION% ...
"%ISCC%" /Q /DAppVersion=%VERSION% "installer\YoRA Slicer.iss" || goto failed
echo.
echo Done: %USERPROFILE%\.yora\dist\YoRA-Slicer-Setup-%VERSION%.exe
exit /b 0

:no_inno
echo Inno Setup 6 is not installed. Install it once with:  winget install JRSoftware.InnoSetup
exit /b 1

:failed
echo The installer build failed, see the messages above.
exit /b 1
