; ============================================================================
;  Fox Mocap — Inno Setup 6 installer script
;
;  Build:
;     ISCC.exe /DAppVersion=0.2.0 /DBuildDir=..\build\bin /DOutDir=..\dist ^
;              installer\fox_mocap.iss
;
;  Required defines (passed via /D from CI or build script):
;     AppVersion  — semantic version, e.g. "0.2.0" (no leading "v")
;     BuildDir    — folder with fox_mocap.exe + Qt/Xsens/Manus DLLs (build/bin)
;     OutDir      — where the resulting Setup .exe is written
;
;  Defaults are provided for local "double-click iscc" testing.
; ============================================================================

#ifndef AppVersion
  #define AppVersion "dev"
#endif

#ifndef BuildDir
  #define BuildDir "..\build\bin"
#endif

#ifndef OutDir
  #define OutDir "..\dist"
#endif

#define MyAppName        "Fox Mocap"
#define MyAppExeName     "fox_mocap.exe"
#define MyAppPublisher   "Fox Mocap contributors"
#define MyAppURL         "https://github.com/ZLYKKIN/Xsens_Fox"
#define MyAppSupportURL  "https://github.com/ZLYKKIN/Xsens_Fox/issues"
#define MyAppUpdatesURL  "https://github.com/ZLYKKIN/Xsens_Fox/releases"

[Setup]
; AppId — stable GUID; do NOT change between versions, otherwise the installer
; will treat new versions as a different product and won't upgrade in place.
AppId={{8E0F2B72-2CDB-4F2A-9F12-A1B2C3D4E5F6}}
AppName={#MyAppName}
AppVersion={#AppVersion}
AppVerName={#MyAppName} {#AppVersion}
AppPublisher={#MyAppPublisher}
AppPublisherURL={#MyAppURL}
AppSupportURL={#MyAppSupportURL}
AppUpdatesURL={#MyAppUpdatesURL}
VersionInfoVersion=0.1.0.0
VersionInfoProductName={#MyAppName}
VersionInfoProductVersion=0.1.0.0
VersionInfoCompany={#MyAppPublisher}

DefaultDirName={autopf}\{#MyAppName}
DefaultGroupName={#MyAppName}
DisableProgramGroupPage=auto
DisableWelcomePage=no
DisableDirPage=no
AllowNoIcons=yes

; Bundle the project licence so the user gets a real EULA step like in games.
LicenseFile=..\LICENSE

OutputDir={#OutDir}
OutputBaseFilename=FoxMocapSetup-{#AppVersion}-windows-x64

Compression=lzma2/ultra64
SolidCompression=yes
LZMAUseSeparateProcess=yes
WizardStyle=modern
WizardResizable=yes

; 64-bit-only; install in 64-bit Program Files even on x64 hosts.
ArchitecturesAllowed=x64
ArchitecturesInstallIn64BitMode=x64

; Default: install for "All Users" (admin) — like most games.
; The user can still pick "Just me" from the dropdown in the privileges page.
PrivilegesRequired=admin
PrivilegesRequiredOverridesAllowed=dialog commandline

UninstallDisplayName={#MyAppName} {#AppVersion}
UninstallDisplayIcon={app}\{#MyAppExeName}
UsePreviousAppDir=yes
UsePreviousGroup=yes
UsePreviousTasks=yes
ChangesAssociations=no
SetupLogging=yes
CloseApplications=force
RestartApplications=no
MinVersion=10.0

[Languages]
Name: "english";  MessagesFile: "compiler:Default.isl"
Name: "russian";  MessagesFile: "compiler:Languages\Russian.isl"

; ----------------------------------------------------------------------------
;  Install types & components — the "do I want the plugins?" picker
; ----------------------------------------------------------------------------
[Types]
Name: "full";    Description: "{cm:FullInstallation}"
Name: "compact"; Description: "{cm:CompactInstallation}"
Name: "custom";  Description: "{cm:CustomInstallation}"; Flags: iscustom

[Components]
Name: "main";              Description: "{cm:CompMain}";           Types: full compact custom; Flags: fixed
Name: "plugins";           Description: "{cm:CompPluginsRoot}";    Types: full
Name: "plugins\blender";   Description: "{cm:CompPluginBlender}";  Types: full
Name: "plugins\unreal";    Description: "{cm:CompPluginUnreal}";   Types: full
; --- Optional hardware drivers (third-party redistributables). Each
;     entry runs the vendor's own signed installer; we do not modify
;     anything inside, just chain-launch it. User picks what they need.
Name: "drivers";              Description: "{cm:CompDriversRoot}";   Types: full
Name: "drivers\usbserial";    Description: "{cm:CompDriverUsbSerial}"; Types: full
Name: "drivers\ethernet";     Description: "{cm:CompDriverEthernet}";  Types: full
Name: "drivers\bonjour";      Description: "{cm:CompDriverBonjour}";   Types: full
Name: "drivers\gigevision";   Description: "{cm:CompDriverGigEVision}"; Types: full
Name: "docs";              Description: "{cm:CompDocs}";           Types: full

[Tasks]
Name: "desktopicon";    Description: "{cm:CreateDesktopIcon}";       GroupDescription: "{cm:AdditionalIcons}"
Name: "quicklaunchicon"; Description: "{cm:CreateQuickLaunchIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked; OnlyBelowVersion: 6.1
Name: "firewall";       Description: "{cm:OpenFirewallUDP9763}";    GroupDescription: "{cm:GroupNetwork}"

; ----------------------------------------------------------------------------
;  Files
; ----------------------------------------------------------------------------
[Files]
; --- main application: fox_mocap.exe + all DLLs + Qt plugins from build/bin/
Source: "{#BuildDir}\*"; DestDir: "{app}"; \
    Excludes: "*.pdb,*.ilk,*.exp,*.lib"; \
    Flags: ignoreversion recursesubdirs createallsubdirs; \
    Components: main

; --- documentation
Source: "..\README.md"; DestDir: "{app}"; Flags: ignoreversion; Components: docs
Source: "..\LICENSE";   DestDir: "{app}"; Flags: ignoreversion; Components: main
Source: "..\image\*";   DestDir: "{app}\image"; Flags: ignoreversion recursesubdirs createallsubdirs; Components: docs

; --- Blender plugin (MVN Live add-on). Ships the "standart" variant; the
;     "logger" variant carries extra diagnostic logging and is dev-only.
Source: "..\Plugins\standart\MVNBlenderPlugin-main\*"; \
    DestDir: "{app}\Plugins\MVNBlenderPlugin-main"; \
    Flags: ignoreversion recursesubdirs createallsubdirs; \
    Components: plugins\blender

; --- Unreal Engine 5.6 LiveLink plugin source (drop into a UE project's
;     Plugins/ folder; UBT builds Binaries/Intermediate on first launch).
Source: "..\Plugins\standart\XsensLivc\LiveLinkMvnPlugin.uplugin"; \
    DestDir: "{app}\Plugins\XsensLivc"; Flags: ignoreversion; \
    Components: plugins\unreal
Source: "..\Plugins\standart\XsensLivc\Source\*"; \
    DestDir: "{app}\Plugins\XsensLivc\Source"; \
    Flags: ignoreversion recursesubdirs createallsubdirs; \
    Components: plugins\unreal
Source: "..\Plugins\standart\XsensLivc\Resources\*"; \
    DestDir: "{app}\Plugins\XsensLivc\Resources"; \
    Flags: ignoreversion recursesubdirs createallsubdirs; \
    Components: plugins\unreal
Source: "..\Plugins\standart\XsensLivc\Content\*"; \
    DestDir: "{app}\Plugins\XsensLivc\Content"; \
    Flags: ignoreversion recursesubdirs createallsubdirs; \
    Components: plugins\unreal

; --- Hardware drivers: each is the original vendor installer, signed
;     by the vendor (Silicon Labs, D-Link, Apple, Allied Vision). The
;     payload is staged into {tmp} so it never lingers under {app}.
Source: ".\drivers\USBXpressInstaller_x64.exe"; \
    DestDir: "{tmp}\drivers"; Flags: deleteafterinstall; \
    Components: drivers\usbserial
Source: ".\drivers\DUB-13X2_USB3_Gigabit_Drivers.msi"; \
    DestDir: "{tmp}\drivers"; Flags: deleteafterinstall; \
    Components: drivers\ethernet
Source: ".\drivers\BonjourPSSetup.exe"; \
    DestDir: "{tmp}\drivers"; Flags: deleteafterinstall; \
    Components: drivers\bonjour
Source: ".\drivers\AlliedVisionGigEFilter_1.22.exe"; \
    DestDir: "{tmp}\drivers"; Flags: deleteafterinstall; \
    Components: drivers\gigevision

[Icons]
Name: "{group}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; WorkingDir: "{app}"
Name: "{group}\Project README"; Filename: "{app}\README.md"; Components: docs
Name: "{group}\GitHub page";   Filename: "{#MyAppURL}"
Name: "{group}\{cm:UninstallProgram,{#MyAppName}}"; Filename: "{uninstallexe}"
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; \
    WorkingDir: "{app}"; Tasks: desktopicon
Name: "{userappdata}\Microsoft\Internet Explorer\Quick Launch\{#MyAppName}"; \
    Filename: "{app}\{#MyAppExeName}"; WorkingDir: "{app}"; Tasks: quicklaunchicon

; ----------------------------------------------------------------------------
;  Post-install actions
; ----------------------------------------------------------------------------
[Run]
; Open Windows Firewall on UDP 9763 (MXTP stream port) so MVN can reach us.
Filename: "{sys}\netsh.exe"; \
    Parameters: "advfirewall firewall add rule name=""Fox Mocap (UDP 9763)"" dir=in action=allow protocol=UDP localport=9763"; \
    Flags: runhidden; Tasks: firewall

; --- Hardware drivers: chain-launch each vendor's installer in turn.
;     We use waituntilterminated so the user finishes one driver wizard
;     before the next opens (otherwise four overlapping UAC prompts).
Filename: "{tmp}\drivers\USBXpressInstaller_x64.exe"; \
    StatusMsg: "{cm:RunDriverUsbSerial}"; \
    Flags: waituntilterminated; Components: drivers\usbserial
Filename: "msiexec.exe"; \
    Parameters: "/i ""{tmp}\drivers\DUB-13X2_USB3_Gigabit_Drivers.msi"" /qb"; \
    StatusMsg: "{cm:RunDriverEthernet}"; \
    Flags: waituntilterminated; Components: drivers\ethernet
Filename: "{tmp}\drivers\BonjourPSSetup.exe"; \
    StatusMsg: "{cm:RunDriverBonjour}"; \
    Flags: waituntilterminated; Components: drivers\bonjour
Filename: "{tmp}\drivers\AlliedVisionGigEFilter_1.22.exe"; \
    StatusMsg: "{cm:RunDriverGigEVision}"; \
    Flags: waituntilterminated; Components: drivers\gigevision

; "Run Fox Mocap now" checkbox on the Finish page.
Filename: "{app}\{#MyAppExeName}"; \
    Description: "{cm:LaunchProgram,{#StringChange(MyAppName, '&', '&&')}}"; \
    Flags: nowait postinstall skipifsilent

[UninstallRun]
Filename: "{sys}\netsh.exe"; \
    Parameters: "advfirewall firewall delete rule name=""Fox Mocap (UDP 9763)"""; \
    Flags: runhidden

; ----------------------------------------------------------------------------
;  Custom translations for component / task descriptions
; ----------------------------------------------------------------------------
[CustomMessages]
; -- English
english.FullInstallation=Full installation (application + plugins + docs)
english.CompactInstallation=Compact installation (application only)
english.CustomInstallation=Custom installation
english.CompMain=Fox Mocap application and runtime DLLs (Xsens / Manus / Qt)
english.CompPluginsRoot=Optional plugins
english.CompPluginBlender=Blender plugin (MVN Live add-on + test project)
english.CompPluginUnreal=Unreal Engine 5.6 plugin source (LiveLink, drop into a UE project)
english.CompDocs=Documentation, screenshots and README
english.CompDriversRoot=Hardware drivers (optional, third-party installers)
english.CompDriverUsbSerial=Silicon Labs USBXpress (CP210x — Awinda USB dongle / MT-Link)
english.CompDriverEthernet=D-Link DUB-13X2 USB 3.0 → Gigabit Ethernet (Awinda station Ethernet)
english.CompDriverBonjour=Apple Bonjour Print Services (mDNS auto-discovery)
english.CompDriverGigEVision=Allied Vision GigE Filter Driver 1.22 (GigE cameras)
english.RunDriverUsbSerial=Installing Silicon Labs USBXpress driver...
english.RunDriverEthernet=Installing D-Link DUB-13X2 driver...
english.RunDriverBonjour=Installing Apple Bonjour service...
english.RunDriverGigEVision=Installing Allied Vision GigE filter driver...
english.CreateQuickLaunchIcon=Create a &Quick Launch icon
english.OpenFirewallUDP9763=Open UDP port 9763 in Windows Firewall (required to receive MVN stream)
english.GroupNetwork=Networking:

; -- Russian
russian.FullInstallation=Полная установка (приложение + плагины + документация)
russian.CompactInstallation=Минимальная (только приложение)
russian.CustomInstallation=Выборочная установка
russian.CompMain=Приложение Fox Mocap и runtime-библиотеки (Xsens / Manus / Qt)
russian.CompPluginsRoot=Дополнительные плагины
russian.CompPluginBlender=Плагин для Blender (MVN Live add-on + тест-проект)
russian.CompPluginUnreal=Плагин для Unreal Engine 5.6 (исходники LiveLink, кладётся в Plugins/ UE-проекта)
russian.CompDocs=Документация, скриншоты и README
russian.CompDriversRoot=Драйверы оборудования (опционально, сторонние установщики)
russian.CompDriverUsbSerial=Silicon Labs USBXpress (CP210x — USB-донгл Awinda / MT-Link)
russian.CompDriverEthernet=D-Link DUB-13X2 USB 3.0 → Gigabit Ethernet (Ethernet станции Awinda)
russian.CompDriverBonjour=Apple Bonjour Print Services (mDNS — авто-обнаружение)
russian.CompDriverGigEVision=Allied Vision GigE Filter Driver 1.22 (камеры GigE)
russian.RunDriverUsbSerial=Установка драйвера Silicon Labs USBXpress…
russian.RunDriverEthernet=Установка драйвера D-Link DUB-13X2…
russian.RunDriverBonjour=Установка службы Apple Bonjour…
russian.RunDriverGigEVision=Установка драйвера Allied Vision GigE…
russian.CreateQuickLaunchIcon=Создать значок в панели &быстрого запуска
russian.OpenFirewallUDP9763=Открыть UDP-порт 9763 в брандмауэре Windows (нужен для приёма потока MVN)
russian.GroupNetwork=Сеть:

; ----------------------------------------------------------------------------
;  Pre-install sanity: refuse to run on 32-bit Windows.
; ----------------------------------------------------------------------------
[Code]
function InitializeSetup(): Boolean;
begin
  if not IsWin64 then
  begin
    MsgBox('Fox Mocap requires a 64-bit version of Windows 10 or later.',
           mbError, MB_OK);
    Result := False;
    exit;
  end;
  Result := True;
end;
