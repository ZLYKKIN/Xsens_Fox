# Fox Mocap

![Fox Mocap — New Session](image/newsession.png)

[![Release](https://img.shields.io/github/v/release/ZLYKKIN/Xsens_Fox?include_prereleases&sort=semver&label=release)](https://github.com/ZLYKKIN/Xsens_Fox/releases/latest)
[![Downloads](https://img.shields.io/github/downloads/ZLYKKIN/Xsens_Fox/total.svg)](https://github.com/ZLYKKIN/Xsens_Fox/releases)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

---

## 🇬🇧 About the project

**Fox Mocap** is a free Windows application for full-body motion capture with an
[Xsens / Movella MVN](https://www.movella.com/products/motion-capture/xsens-mvn-link)
inertial suit. It takes the raw sensor stream straight off the suit, builds a clean
17-sensor / 23-segment skeleton, shows it live in its own 3-D viewport, and streams the
exact same pose into **Unreal Engine 5.6** and **Blender** in real time — no MVN Live
licence required. Optional **Manus** gloves add full finger tracking.

You put on the suit, run a short three-pose calibration, press *Start*, and your
character moves with you everywhere at once: in the app's viewport, in Unreal, and in
Blender.

**How it works under the hood:**

1. **Receive** — the suit's inertial data arrives over UDP (Movella MXTP02 / MXTP25).
   Update rate follows the hardware automatically — 240 Hz for MVN Link, 60 Hz for Awinda.
2. **Fuse** — each sensor runs through an [xio Fusion](https://github.com/xioTechnologies/Fusion)
   AHRS (Madgwick filter + gyro-bias estimator) to produce a stable orientation, hardened
   against NaN / dropouts at every external boundary.
3. **Calibrate** — a guided **T-pose → N-pose → K-pose** sequence solves the
   sensor-to-segment alignment, including asymmetric and mirrored sensor mounts.
4. **Solve the skeleton** — forward kinematics drives all 23 segments (spine, arms, legs,
   articulated feet/toes) with anatomical joint limits; locomotion logic keeps the pelvis
   anchored and kills foot-sliding and vertical drift.
5. **Render & stream** — the pose is drawn in an OpenGL window and streamed simultaneously
   to Unreal and Blender from a single shared world-frame, so every viewer agrees.
6. **Record** — capture takes to BVH / FBX for editing later.

The whole interface is fully localized in **English and Russian**.

## 🇷🇺 О проекте

**Fox Mocap** — это бесплатное приложение для Windows для захвата движений всего тела с
инерциального костюма [Xsens / Movella MVN](https://www.movella.com/products/motion-capture/xsens-mvn-link).
Программа принимает поток данных прямо с костюма, строит чистый скелет из 17 сенсоров и
23 сегментов, показывает его вживую в собственном 3-D окне и одновременно транслирует ту же
позу в **Unreal Engine 5.6** и **Blender** в реальном времени — лицензия MVN Live не нужна.
Опционально перчатки **Manus** добавляют полный трекинг пальцев.

Надеваете костюм, проходите короткую калибровку из трёх поз, нажимаете *Start* — и ваш
персонаж двигается вместе с вами сразу везде: во вьюпорте приложения, в Unreal и в Blender.

**Как это работает внутри:**

1. **Приём** — инерциальные данные костюма приходят по UDP (Movella MXTP02 / MXTP25).
   Частота подстраивается под оборудование автоматически — 240 Гц для MVN Link, 60 Гц для Awinda.
2. **Фьюжн** — каждый сенсор проходит через AHRS-фильтр [xio Fusion](https://github.com/xioTechnologies/Fusion)
   (фильтр Маджвика + оценка дрейфа гироскопа) для получения стабильной ориентации, с защитой
   от NaN и пропусков на всех внешних границах.
3. **Калибровка** — последовательность **T-поза → N-поза → K-поза** вычисляет привязку
   сенсоров к сегментам, включая случаи зеркальной и несимметричной установки датчиков.
4. **Решение скелета** — прямая кинематика управляет всеми 23 сегментами (позвоночник, руки,
   ноги, артикулированные стопы и носки) с анатомическими ограничениями суставов; логика
   локомоции удерживает таз на месте и убирает проскальзывание стоп и вертикальный дрейф.
5. **Отрисовка и стриминг** — поза рисуется в окне OpenGL и одновременно отправляется в Unreal
   и Blender из единой мировой системы координат, поэтому все вьюверы показывают одно и то же.
6. **Запись** — съёмка дублей в BVH / FBX для последующего монтажа.

Весь интерфейс полностью переведён на **английский и русский** языки.

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
