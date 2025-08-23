; Inno Setup script for Scenario Inspector
; Save as scenario_inspector.iss and compile with Inno Setup Compiler

[Setup]
AppName=Scenario Inspector
AppVersion=1.0
DefaultDirName={autopf}\ScenarioInspector
DefaultGroupName=Scenario Inspector
UninstallDisplayIcon={app}\Scenario_Inspector.exe
OutputDir=dist
OutputBaseFilename=ScenarioInspectorSetup
Compression=lzma
SolidCompression=yes

[Files]
Source: "dist\Scenario_Inspector.exe"; DestDir: "{app}"; Flags: ignoreversion
Source: "programs\*"; DestDir: "{app}\programs"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "logs\*"; DestDir: "{app}\logs"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "output\*"; DestDir: "{app}\output"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "libs\*"; DestDir: "{app}\libs"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "config\*"; DestDir: "{app}\config"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{group}\Scenario Inspector"; Filename: "{app}\Scenario_Inspector.exe"
Name: "{commondesktop}\Scenario Inspector"; Filename: "{app}\Scenario_Inspector.exe"; Tasks: desktopicon

[Tasks]
Name: "desktopicon"; Description: "Create a &desktop icon"; GroupDescription: "Additional icons:"

[Run]
Filename: "{app}\Scenario_Inspector.exe"; Description: "Launch Scenario Inspector"; Flags: nowait postinstall skipifsilent
