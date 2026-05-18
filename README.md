# Fox Mocap

![Fox Mocap — New Session](image/newsession.png)

**Fox Mocap** is a free Windows app that captures full-body motion from an
[Xsens MVN](https://www.movella.com/products/motion-capture/xsens-mvn-link)
inertial suit and streams it live into **Unreal Engine** and **Blender**.

Plug in your suit, run the wizard, hit *Start* — your character moves with you.

[![Release](https://img.shields.io/github/v/release/ZLYKKIN/Xsens_Fox?include_prereleases&sort=semver&label=release)](https://github.com/ZLYKKIN/Xsens_Fox/releases/latest)
[![Downloads](https://img.shields.io/github/downloads/ZLYKKIN/Xsens_Fox/total.svg)](https://github.com/ZLYKKIN/Xsens_Fox/releases)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

---

## Download

Grab the latest Windows build from the
[**Releases**](https://github.com/ZLYKKIN/Xsens_Fox/releases/latest) page:

* **`FoxMocapSetup-*.exe`** — installer with a wizard (recommended).
* `fox_mocap-*.zip` — portable, just unzip and run.

---

## What it does

* Receives the live MVN suit stream over UDP (no MVN Live add-on needed).
* Calibrates in three poses — **T-pose → N-pose → K-pose** — for a clean rig.
* Renders a live 3-D skeleton in its own OpenGL window.
* Streams the same pose to **Unreal Engine 5.6** and **Blender** in real time.
* Optional **Manus** glove support for finger tracking.
* Records sessions to BVH / FBX.

---

## How to use it

1. Put on the Xsens suit, power up the Awinda station, start MVN Animate / Analyze.
2. In MVN turn on **Network Streamer → UDP** to `127.0.0.1:9763` (format MXTP02 or MXTP25).
3. *(Optional)* Plug in Manus gloves, start Manus Core.
4. Launch **Fox Mocap**.
5. Click **New Session** → run calibration (T → N → K poses).
6. Hit **Live → Start**. Your skeleton renders in the viewport and streams out
   to UE / Blender at the same time.

Reference shots of each pose live under [`image/`](image/).

---

## Build from source

Need Windows 10/11 x64, **Visual Studio 2022 Build Tools**, **Qt 6.5.3**
(`msvc2019_64`), CMake 3.21+ and Ninja.

```bat
build.bat
```

`build.bat` calls `vcvars64.bat`, runs CMake with Ninja and ends with
`build\bin\fox_mocap.exe` ready to launch (all Qt and Xsens / Manus DLLs
are staged next to it automatically).

If your Qt lives somewhere other than `C:\Qt\6.5.3\msvc2019_64`, edit the
`QT_DIR` line at the top of `build.bat` first.

---

## Bundled plugins

* **Blender** — MVN Live add-on (`Plugins/MVNBlenderPlugin-main.zip`) +
  a tiny test project under `Plugins/BlenderProject/`.
* **Unreal Engine 5.6** — LiveLink-compatible plugin source in
  `Plugins/XsensLivc/`. Drop it into a UE project's `Plugins/` folder and
  rebuild.

Both are opt-in checkboxes on the installer's *Components* page.

---

## Bundled hardware drivers (optional)

The installer can also chain-launch four signed third-party driver setups
straight from the Components page — pick what your rig needs, leave the
rest unchecked:

* **Silicon Labs USBXpress** (CP210x — Awinda USB dongle / MT-Link).
* **D-Link DUB-13X2** (USB 3.0 → Gigabit Ethernet bridge for Awinda Station).
* **Apple Bonjour** (mDNS auto-discovery).
* **Allied Vision GigE Filter Driver 1.22** (GigE Vision cameras).

Each is the vendor's own signed installer — Fox Mocap just runs them.

---

## License

**MIT** — see [`LICENSE`](LICENSE). Third-party runtime components
(Xsens / Movella, Manus, Qt 6, xio Fusion) keep their original licences.

---

## Maintainer note

> Real life keeps me from giving Fox Mocap as much time as it deserves
> right now, so the project moves in slow bursts rather than a steady
> stream. If you stumble on a bug or quirk — please still open an issue
> or, even better, a small focused pull request with a fix. I read
> everything and merge solid contributions even when I cannot drive new
> features myself. Thank you for understanding!

---

## Contact

- GitHub issues / PRs: [ZLYKKIN/Xsens_Fox](https://github.com/ZLYKKIN/Xsens_Fox/issues)
- Telegram: [**@ZLYKKIN_FOX**](https://t.me/ZLYKKIN_FOX) — quick questions, mocap-rig advice, bug reports that don't fit a GitHub issue
