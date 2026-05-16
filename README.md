# Fox Mocap

![Fox Mocap — New Session](image/newsession.png)

**Fox Mocap** is an MVN-compatible motion-capture application for the
[Xsens MVN](https://www.movella.com/products/motion-capture/xsens-mvn-link)
inertial body suit (Link / Awinda) and the optional
[Manus](https://www.manus-meta.com/) glove line-up.

It receives a live MVN body stream over UDP `:9763` (formats **MXTP02** and
**MXTP25**), runs a full 23-segment inertial-pose pipeline in real time,
renders a skeleton in an OpenGL viewport, and streams the resulting pose
straight into **Unreal Engine** and **Blender** for previs, motion-design,
animation and virtual-production work.

[![Release](https://img.shields.io/github/v/release/ZLYKKIN/Xsens_Fox?include_prereleases&sort=semver&label=release)](https://github.com/ZLYKKIN/Xsens_Fox/releases/latest)
[![Downloads](https://img.shields.io/github/downloads/ZLYKKIN/Xsens_Fox/total.svg)](https://github.com/ZLYKKIN/Xsens_Fox/releases)
[![CI](https://github.com/ZLYKKIN/Xsens_Fox/actions/workflows/release.yml/badge.svg)](https://github.com/ZLYKKIN/Xsens_Fox/actions/workflows/release.yml)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

---

## Download

Pre-built Windows binaries are published on the
[**Releases**](https://github.com/ZLYKKIN/Xsens_Fox/releases/latest)
page. Two flavours are available for every tag:

| File | When to pick it |
|------|-----------------|
| **`FoxMocapSetup-vX.Y.Z-windows-x64.exe`** | Recommended. A game-style Inno Setup wizard — choose install folder, accept the MIT licence, pick which components to install (app, Blender plugin, Unreal Engine plugin source, docs), optional Start-Menu / desktop shortcuts, and an optional Windows-Firewall rule for UDP `9763`. Clean uninstaller included. |
| `fox_mocap-vX.Y.Z-windows-x64.zip` | Portable. Unpack anywhere and run `fox_mocap.exe`. Convenient for USB drives, dev sandboxes, or running multiple versions side by side. |
| `*.sha256` | SHA-256 checksums to verify the downloads. |

Both archives ship the same core: `fox_mocap.exe`, the Qt 6 runtime
deployed by `windeployqt`, and the Xsens / Manus driver DLLs.

> Windows SmartScreen may warn about an unknown publisher — the binary
> is not code-signed (typical for open-source desktop tools). Compare
> the SHA-256 with the attached `.sha256` file, then click *More info*
> → *Run anyway*.

---

## Features

### Body capture

- **MXTP02 / MXTP25** UDP receiver on port `9763` — no MVN Live add-on
  required, raw MVN stream is consumed directly.
- **23-segment Xsens skeleton** with full-body inertial fusion driven by
  the vendored
  [xio Fusion AHRS](https://github.com/xioTechnologies/Fusion) filter
  (`scr/fusion/`).
- **Three-pose calibration** (**T-pose → N-pose → K-pose**) for robust
  estimation of mounting roll and per-sensor magnetometer normalisation,
  including a TRIAD-based fix when the arms and upper legs see
  near-perpendicular gravity (K-pose: squat + arms forward).
- **Forward kinematics** with sensible default segment lengths derived
  from the actor's height and foot length (standard Drillis-Contini
  anthropometric ratios).
- **Sensor-to-segment alignment** estimated from the calibration poses
  and applied to every raw IMU sample before AHRS.
- **Locomotion solver** — foot-contact detection, foot-lock IK and a
  loose, Kalman-style drift limiter for the pelvis anchor.
- **Scene-yaw alignment** to bring the actor's facing direction to the
  scene's forward axis with a single button.

### Hands

- Optional **Manus** glove integration (`ManusSDK.dll` + `ManusHid.dll`,
  loaded dynamically, no SDK headers required to build).
- Anatomy-aware finger mapping with per-joint flex / spread limits and
  thumb-specific handling.
- Wrist fusion that combines suit forearm orientation with glove wrist
  swing-twist to keep gimbal-free wrist motion.

### Streaming

- **Unreal Engine** — bundled LiveLink-compatible plugin source
  (`Plugins/XsensLivc/`) you can drop into a UE 5.6 project.
- **Blender** — MVN Live add-on (`Plugins/MVNBlenderPlugin-main.zip`)
  plus a tiny ready-to-run scene
  (`Plugins/BlenderProject/testproject.blend`) and a Python script that
  rebuilds the scene from scratch in headless mode for cross-log
  verification against Fox Mocap.
- Output frames go out as MVN-compatible UDP packets — anything that
  accepts MVN Live should pick them up.

### UI

- Qt 6 desktop application, native Windows 11 dark title bar via
  `dwmapi`.
- OpenGL viewport with the live skeleton, contact-state indicators and
  a rec/stream HUD overlay.
- Per-sensor status panel showing per-tracker freshness, last update
  and battery level of the body pack.

---

## Requirements

| Component | Version | Why |
|-----------|---------|-----|
| Windows | 10 / 11 x64 | Xsens / Manus runtime DLLs are Windows-only |
| Visual Studio 2022 Build Tools | MSVC 19.3x | `cl.exe` invoked via `vcvars64.bat` |
| Qt | **6.5.3** (`msvc2019_64`) | `Core`, `Gui`, `Widgets`, `OpenGL`, `OpenGLWidgets`, `Network` |
| CMake | 3.21+ | `find_package(Qt6)` + Ninja generator |
| Ninja | 1.10+ | Build driver |
| Xsens MVN Animate / Analyze | any release that supports MXTP02 or MXTP25 | Data source |
| Manus Core | current | Required only if you use Manus gloves |
| Unreal Engine | 5.6 | Required only if you install the bundled UE plugin |

The driver DLLs (`xsensdeviceapi64.dll`, `xstypes64.dll`,
`ManusSDK.dll`, `ManusHid.dll`, `manus.dll`, `libusb-1.0.dll`,
`libiomp5md.dll`) live in `dll/` and are copied next to the executable
automatically by CMake's `POST_BUILD` step. They are loaded dynamically
via `LoadLibraryEx`, so the project builds without their SDK headers
being present.

---

## Build from source

### One-line build (Windows)

```bat
build.bat
```

`build.bat` invokes `vcvars64.bat`, runs CMake with Ninja and produces
`build\bin\fox_mocap.exe`. Edit the script first if your Qt install
lives somewhere other than `C:\Qt\6.5.3\msvc2019_64`:

```bat
set "QT_DIR=C:\Qt\6.5.3\msvc2019_64"
```

### Manual build

```bat
:: Visual Studio environment
"C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

cmake -S . -B build -G Ninja ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DCMAKE_PREFIX_PATH="C:\Qt\6.5.3\msvc2019_64"

cmake --build build --config Release -j
```

CMake stages the Xsens / Manus DLLs and runs `windeployqt` against the
output, so `build\bin\fox_mocap.exe` is ready to run as soon as the
build finishes — no extra "install" step.

---

## Run

1. Power the Xsens Awinda station, dress the actor in the suit and
   start MVN Animate / Analyze.
2. Enable the MVN Network Streamer → UDP, target `127.0.0.1:9763`,
   format `MXTP02` (or `MXTP25` if you want finger segments).
3. *(Optional)* Plug in the Manus gloves and start Manus Core.
4. Launch `fox_mocap.exe`.
5. Click **New Session** and walk through calibration:
   - **T-pose** — arms out to the sides, palms down.
   - **N-pose** — arms by the body, palms facing the thighs.
   - **K-pose** — half-squat with arms pointed straight forward
     (provides perpendicular gravity for arms / upper legs so the TRIAD
     branch kicks in).
6. Hit **Live → Start**. The skeleton renders in the viewport and is
   streamed out to UE / Blender simultaneously.

Reference images for each pose live under `image/`:
[`image/t-pose.png`](image/t-pose.png),
[`image/npose.png`](image/npose.png),
[`image/k-pose.png`](image/k-pose.png),
[`image/newsession.png`](image/newsession.png).

---

## Bundled plugins

### Unreal Engine — `Plugins/XsensLivc/`

A LiveLink-compatible plugin for UE 5.6 (`LiveLinkMvnPlugin.uplugin`,
three modules: `LiveLinkMvnPlugin`, `LiveLinkMvnPluginEditor`,
`XSensLLWrapper`). Drop the folder into your project's `Plugins/`
directory and rebuild the project — the LiveLink source will then
appear under *Window → Virtual Production → Live Link*.

Pre-built `Binaries/` and `Intermediate/` are intentionally **not**
committed; Unreal will produce them on first build for your engine
version.

### Blender — `Plugins/MVNBlenderPlugin-main.zip` and `Plugins/BlenderProject/`

- `MVNBlenderPlugin-main.zip` — the MVN Live add-on, install via
  *Edit → Preferences → Add-ons → Install…* and point it at the ZIP.
- `BlenderProject/testproject.blend` — minimal scene wired up for live
  retargeting against Fox Mocap.
- `BlenderProject/setup_testproject.py` — rebuilds that scene from
  scratch in headless mode (useful for CI / cross-log verification
  against the Fox Mocap output stream).
- `BlenderProject/README.txt` — step-by-step instructions for the diff-
  log verification workflow.

---

## Project layout

```
Xsens_Fox/
├─ CMakeLists.txt           # Qt 6 + Ninja build
├─ build.bat                # One-shot Windows build (vcvars64 + cmake + ninja)
├─ LICENSE                  # MIT
├─ scr/                     # Sources
│   ├─ main.cpp             # ~10 kLOC: GUI, fusion, calibration, FK, streaming
│   ├─ main.h               # Types, constants, declarations
│   ├─ resources.qrc        # Qt resource bundle (image/)
│   └─ fusion/              # Vendored xio Fusion AHRS (MIT)
├─ dll/                     # Xsens / Manus runtime DLLs
├─ image/                   # Calibration-pose screenshots + UI assets
├─ installer/               # Inno Setup 6 script (used by Release CI)
├─ Plugins/
│   ├─ BlenderProject/      # Test scene + headless setup script
│   ├─ MVNBlenderPlugin-main.zip
│   └─ XsensLivc/           # UE 5.6 LiveLink plugin source (Source/Content/Resources)
└─ .github/workflows/       # GitHub Actions: build, package, publish release
```

---

## Releases & CI

Every push of a `v*` tag triggers the
[`Release`](.github/workflows/release.yml) workflow, which:

1. Spins up a `windows-2022` runner.
2. Installs Qt 6.5.3 (`msvc2019_64`) via `jurplel/install-qt-action`
   and brings in MSVC + Ninja.
3. Configures + builds the project with CMake / Ninja and runs
   `windeployqt`.
4. Packages a portable `*.zip` and an installer `*.exe` (Inno Setup 6).
5. Computes `*.sha256` for both.
6. Publishes everything to the GitHub Release attached to that tag.

You can cut a new release by pushing a `vX.Y.Z` tag, or trigger a
test build manually from
*Actions → Release → Run workflow*.

---

## License

This project is released under the **MIT License** — see
[`LICENSE`](LICENSE) for the full text. The MIT terms apply to the
Fox Mocap source code in this repository (`scr/`, `installer/`,
`Plugins/BlenderProject/setup_testproject.py`).

Third-party components shipped with the application keep their own
licences:

- **xio Fusion AHRS** (`scr/fusion/`) — MIT, © x-io Technologies.
- **Qt 6 runtime** (deployed by `windeployqt`) — LGPLv3, sources
  available on [download.qt.io](https://download.qt.io/).
- **Xsens Device API runtime** (`dll/xsensdeviceapi64.dll`,
  `xstypes64.dll`) — proprietary, redistributed as a runtime under the
  Movella / Xsens EULA.
- **Manus SDK & HID runtime** (`dll/ManusSDK.dll`, `manus.dll`,
  `ManusHid.dll`, `libusb-1.0.dll`, `libiomp5md.dll`) — proprietary,
  redistributed under the Manus Meta EULA.
- **MVN Blender plugin** (`Plugins/MVNBlenderPlugin-main.zip`) —
  authored by Movella, distributed under its original licence.
- **MVN Unreal Engine plugin** (`Plugins/XsensLivc/`) — authored by
  Movella Technologies B.V. (`LiveLinkMvnPlugin`), distributed under
  its original licence.

If you intend to redistribute Fox Mocap together with the proprietary
runtimes above, make sure your distribution channel is covered by the
respective vendor EULAs.

---

## Contributing

> A short note from the maintainer
>
> Real life keeps me from giving Fox Mocap as much time as it deserves
> right now, so the project moves in slow bursts rather than a steady
> stream. If you stumble on a bug, a quirk, or a regression — please
> still open an issue or, even better, a pull request with a fix. I
> read everything and merge solid contributions even when I cannot
> drive new features myself. Thank you for understanding!

Issues and pull requests are welcome on
[GitHub](https://github.com/ZLYKKIN/Xsens_Fox/issues).

When opening a PR:

- Keep the build green (`build.bat` should produce a working
  `fox_mocap.exe`).
- Match the existing C++20 / Qt 6 style; no exceptions across the
  fusion / receiver / FK boundaries.
- Mention any non-trivial mocap-rig assumption you rely on, so future
  readers can reproduce the experiment.

Bug-fix PRs are especially welcome — small, focused patches that fix
a single reproducible issue get merged the fastest.

---

## Contact

- GitHub issues / PRs: [ZLYKKIN/Xsens_Fox](https://github.com/ZLYKKIN/Xsens_Fox/issues)
- Telegram: [**@ZLYKKIN_FOX**](https://t.me/ZLYKKIN_FOX) — quick questions, mocap-rig advice, bug reports that don't fit a GitHub issue
