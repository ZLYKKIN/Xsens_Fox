# Xsens Fox

Fox Mocap — Qt-приложение для motion capture на базе **Xsens Link (MVN MXTP02/25)** костюма и опциональных перчаток **Manus**. Полный C++ порт `HumanInertialPose-main` (23-сегментный Xsens-скелет, T/N/K-pose калибровка, прямая кинематика, sensor-to-segment alignment), стрим в Unreal Engine / Blender по UDP `9763`.

> ⚠️ Репозиторий приватный. Открыт он будет позже — после публикации это README станет «лицом» проекта.

---

## Возможности

- Приём **MXTP02 / MXTP25** пакетов от Xsens MVN Awinda / Link напрямую по UDP `9763`.
- 23-сегментный скелет с full-body inertial fusion (xio Fusion AHRS, vendored через XESNSE).
- Калибровка в трёх позах: **T-pose → N-pose → K-pose** (присед + руки вперёд) для устойчивого определения mounting roll.
- Поддержка перчаток **Manus** (HID + ManusSDK) — finger tracking с правильным mapping (thumb / spread anatomy).
- Realtime forward kinematics, scene-yaw alignment, локомоция с heel-strike detection.
- OpenGL viewport на Qt 6, тёмная тема Windows 11.
- Стрим позы в **Unreal Engine** и **Blender** (MVN Live Plugin совместимый формат).

---

## Требования

| Компонент | Версия | Зачем |
|-----------|--------|-------|
| Windows | 10 / 11 x64 | Только Windows (Xsens / Manus SDK Windows-only) |
| Visual Studio 2022 Build Tools | MSVC 19.3x | `cl.exe` через `vcvars64.bat` |
| Qt | **6.5.3** (msvc2019_64) | Core, Gui, Widgets, OpenGL, OpenGLWidgets, Network |
| CMake | 3.21+ | `find_package(Qt6)` + Ninja генератор |
| Ninja | 1.10+ | Быстрая сборка |
| Xsens MVN Analyze / Animate | любая поддерживающая MXTP02/25 | Источник данных |
| Manus Core (опционально) | актуальная | Если используете перчатки |

Драйвера `xsensdeviceapi64.dll`, `xstypes64.dll`, `ManusSDK.dll`, `ManusHid.dll`, `manus.dll`, `libusb-1.0.dll`, `libiomp5md.dll` лежат в `dll/` и автоматически копируются в `build/bin/` рядом с `.exe` через `add_custom_command POST_BUILD` (см. `CMakeLists.txt`).

---

## Сборка

### Быстрый старт (Windows, готовый скрипт)

```bat
build.bat
```

`build.bat` сам зовёт `vcvars64.bat`, конфигурирует CMake и запускает Ninja. Перед запуском подправьте путь к Qt, если ставили не в стандартное место:

```bat
set "QT_DIR=C:\Qt\6.5.3\msvc2019_64"
```

### Ручная сборка

```bat
:: Visual Studio environment
"C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

cmake -S . -B build -G Ninja ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DCMAKE_PREFIX_PATH="C:\Qt\6.5.3\msvc2019_64"

cmake --build build --config Release -j
```

Готовый бинарник: `build\bin\fox_mocap.exe`. Туда же CMake копирует все DLL из `dll/` и запускает `windeployqt`, так что приложение готово к запуску из коробки.

---

## Запуск

1. Включите Xsens Awinda station, наденьте костюм, запустите MVN Analyze / Animate.
2. В MVN включите **Network Streamer** → UDP, target `127.0.0.1:9763`, формат `MXTP02` (или `MXTP25` с финковыми сегментами).
3. (Опционально) Подключите перчатки Manus и запустите Manus Core.
4. Запустите `fox_mocap.exe`.
5. Нажмите **New Session** → выполните калибровку:
   - **T-pose** — руки в стороны, ладони вниз.
   - **N-pose** — руки вдоль тела, ладони к бёдрам.
   - **K-pose** — присед + руки прямо вперёд (даёт perpendicular gravity для arms / upper_legs → TRIAD активен).
6. После калибровки — **Live → Start**. Скелет начинает рендериться в OpenGL viewport и одновременно стримится в UE / Blender.

Изображения каждой позы — `image/t-pose.png`, `image/npose.png`, `image/k-pose.png`, `image/newsession.png`.

---

## Стрим в Blender

В `Plugins/` лежит:

- `MVNBlenderPlugin-main.zip` — плагин для Blender (MVN Live).
- `BlenderProject/` — тестовый `.blend` + headless setup script + README с описанием cross-log verification против Fox Mocap.

См. `Plugins/BlenderProject/README.txt` для инструкций по проверке стрима через diff логов.

---

## Структура проекта

```
Xsens_Fox/
├─ CMakeLists.txt          # Qt6 + Ninja build
├─ build.bat               # one-shot Windows build
├─ scr/                    # исходники
│   ├─ main.cpp            # ~10K строк: GUI, fusion, calibration, kinematics, stream
│   ├─ main.h              # типы, константы, объявления
│   ├─ resources.qrc       # Qt resources (image/)
│   └─ fusion/             # xio Fusion AHRS (vendored)
├─ dll/                    # Xsens / Manus runtime DLL
├─ image/                  # калибровочные позы + UI
└─ Plugins/                # Blender плагин + тест-проект
```

---

## Лицензия

TBD. До публикации репозитория — внутреннее использование. После открытия будет добавлен `LICENSE` (планируется MIT, но финальное решение остаётся за автором).

---

## Контакты

Issues / PR — после открытия репозитория. Сейчас по приватному каналу.
