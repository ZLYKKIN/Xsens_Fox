@echo off
setlocal
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 (
  echo [ERROR] vcvars64.bat failed
  exit /b 1
)

set "QT_DIR=C:\Qt\6.5.3\msvc2019_64"
set "PATH=%QT_DIR%\bin;%PATH%"

cd /d "%~dp0"

if not exist build mkdir build

echo === configure ===
cmake -S . -B build -G Ninja ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DCMAKE_PREFIX_PATH="%QT_DIR%" ^
  -DCMAKE_C_COMPILER=cl ^
  -DCMAKE_CXX_COMPILER=cl
if errorlevel 1 exit /b 1

echo === build ===
cmake --build build --config Release -j
if errorlevel 1 exit /b 1

echo === math tests ===
where python >nul 2>&1
if errorlevel 1 (
  echo [skip] python not on PATH — skipping tests/python/run_all.py
) else (
  python tests\python\run_all.py
  if errorlevel 1 (
    echo [ERROR] python math regression tests failed
    exit /b 1
  )
)

echo.
echo === done ===
dir build\bin\fox_mocap.exe 2>nul
