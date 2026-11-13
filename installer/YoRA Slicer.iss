; YoRA Slicer installer for Windows (Inno Setup 6). Built by "Build setup.bat", which first builds the program folder
; (%USERPROFILE%\.yora\dist\YoRA Slicer) and passes the version: ISCC /DAppVersion=x.y.z "installer\YoRA Slicer.iss"
;
; It installs for the current user without admin rights (or for everyone, chosen in the first dialog), with a Start
; menu entry and an uninstaller. The user's own files (robots, tools, materials, posts, profiles) live in
; %USERPROFILE%\.yora, so updating or uninstalling the program never touches them.

#ifndef AppVersion
  #define AppVersion "0.0.0"
#endif
#define AppName "YoRA Slicer"
#define AppExe "YoRA Slicer.exe"
#define DistDir GetEnv("USERPROFILE") + "\.yora\dist\YoRA Slicer"

[Setup]
; the same AppId for every version, so a new setup updates the installed one
AppId={{9A99E2A6-D964-44DC-B4BC-0D77C853F631}
AppName={#AppName}
AppVersion={#AppVersion}
AppVerName={#AppName} {#AppVersion}
AppPublisher=njmrvn
AppPublisherURL=https://github.com/njmrvn/YoRA-Slicer
AppSupportURL=https://github.com/njmrvn/YoRA-Slicer/issues
AppUpdatesURL=https://github.com/njmrvn/YoRA-Slicer/releases
DefaultDirName={autopf}\{#AppName}
DisableProgramGroupPage=yes
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog
LicenseFile=..\LICENSE
OutputDir={#GetEnv("USERPROFILE")}\.yora\dist
OutputBaseFilename=YoRA-Slicer-Setup-{#AppVersion}
Compression=lzma2/max
SolidCompression=yes
WizardStyle=modern
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
UninstallDisplayIcon={app}\{#AppExe}
UninstallDisplayName={#AppName}
CloseApplications=yes

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked

[InstallDelete]
; a new version replaces the old program files completely
Type: filesandordirs; Name: "{app}\_internal"

[Files]
Source: "{#DistDir}\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{autoprograms}\{#AppName}"; Filename: "{app}\{#AppExe}"
Name: "{autodesktop}\{#AppName}"; Filename: "{app}\{#AppExe}"; Tasks: desktopicon

[Run]
Filename: "{app}\{#AppExe}"; Description: "{cm:LaunchProgram,{#AppName}}"; Flags: nowait postinstall skipifsilent
