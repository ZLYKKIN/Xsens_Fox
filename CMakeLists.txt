cmake_minimum_required(VERSION 3.21)
project(fox_mocap LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTORCC ON)

# ---- Version stamp from git tag --------------------------------------------
# `git describe --tags --always --dirty` gives us the shortest unique identifier:
#   * exact tag        -> "v0.6.0"
#   * tag + extra      -> "v0.6.0-3-gabcdef1"
#   * dirty working    -> "v0.6.0-3-gabcdef1-dirty"
#   * pre-CI checkout  -> short SHA fallback
# The result is stamped into the binary via FOX_VERSION_STR.
execute_process(
    COMMAND git describe --tags --always --dirty
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    OUTPUT_VARIABLE FOX_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
if (NOT FOX_VERSION)
    set(FOX_VERSION "dev")
endif()
# Strip the leading "v" so QApplication::applicationVersion() looks clean
# in About boxes and crash dumps.
string(REGEX REPLACE "^v" "" FOX_VERSION_CLEAN "${FOX_VERSION}")
message(STATUS "Fox Mocap version: ${FOX_VERSION_CLEAN}")

# All binaries land in <project>/build/bin
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/build/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/build/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/build/lib")

# ---- Qt 6 -------------------------------------------------------------------
# If CMake doesn't find Qt automatically, pass one of these on the command line:
#   -DCMAKE_PREFIX_PATH="C:/Qt/6.7.2/msvc2019_64"
#   -DQt6_DIR="C:/Qt/6.7.2/msvc2019_64/lib/cmake/Qt6"
find_package(Qt6 REQUIRED COMPONENTS
    Core Gui Widgets OpenGL OpenGLWidgets Network)

# ---- Eigen 3 (header-only) --------------------------------------------------
# Strategy: try system/installed Eigen first; if absent, vendor it via
# FetchContent (downloads to build/_deps on first configure, then cached).
# Code can include with either:
#   #include <Eigen/Dense>          (preferred, version-agnostic)
#   #include <eigen3/Eigen/Dense>   (legacy Debian-style)
# Always link via target: target_link_libraries(target PRIVATE Eigen3::Eigen)
find_package(Eigen3 3.4 QUIET NO_MODULE)
if (NOT Eigen3_FOUND)
    message(STATUS "Eigen3 not installed; fetching via FetchContent")
    include(FetchContent)
    FetchContent_Declare(eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG        3.4.0
        GIT_SHALLOW    TRUE)
    # IMPORTANT: do NOT add_subdirectory the Eigen tree — its blas/lapack
    # subdirs `enable_language(Fortran)` which breaks CI Windows runners
    # where gfortran is mingw and the toolchain otherwise is MSVC.
    # Eigen is header-only; an INTERFACE target on its include dir is
    # everything any consumer needs.
    FetchContent_GetProperties(eigen)
    if (NOT eigen_POPULATED)
        FetchContent_Populate(eigen)
    endif()
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    target_include_directories(Eigen3::Eigen INTERFACE "${eigen_SOURCE_DIR}")
endif()
message(STATUS "Eigen3 ready")

# ---- Fox Mocap executable ---------------------------------------------------
add_executable(fox_mocap WIN32
    scr/main.cpp
    scr/main.h
    scr/foxmath.cpp
    scr/foxmath.h
    scr/foxwire.cpp
    scr/foxwire.h
    scr/resources.qrc
    scr/fusion/Fusion.h
    scr/fusion/FusionAhrs.c
    scr/fusion/FusionAhrs.h
    scr/fusion/FusionBias.c
    scr/fusion/FusionBias.h
    scr/fusion/FusionCompass.c
    scr/fusion/FusionCompass.h
    scr/fusion/FusionConvention.h
    scr/fusion/FusionMath.h
    scr/fusion/FusionModel.h
    scr/fusion/FusionRemap.h
)
target_include_directories(fox_mocap PRIVATE scr scr/fusion)
target_compile_definitions(fox_mocap PRIVATE
    FOX_VERSION_STR="${FOX_VERSION_CLEAN}")
target_link_libraries(fox_mocap PRIVATE
    Qt6::Core Qt6::Gui Qt6::Widgets
    Qt6::OpenGL Qt6::OpenGLWidgets
    Qt6::Network
    Eigen3::Eigen)

# Use legacy GL functions (lines/points for skeleton rendering).
# dwmapi — for Windows 11 immersive dark title bar.
if (WIN32)
    target_link_libraries(fox_mocap PRIVATE opengl32 dwmapi)
endif()

# ---- Copy DLLs next to the executable so it can run from build/bin ---------
set(FOX_DLL_SRC "${CMAKE_SOURCE_DIR}/dll")
set(FOX_DLL_DST "${CMAKE_SOURCE_DIR}/build/bin")
if (EXISTS "${FOX_DLL_SRC}")
    add_custom_command(TARGET fox_mocap POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "${FOX_DLL_DST}"
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${FOX_DLL_SRC}" "${FOX_DLL_DST}"
        COMMENT "Staging Xsens/Manus DLLs into build/bin")
endif()

# ---- windeployqt -----------------------------------------------------------
if (WIN32)
    find_program(WINDEPLOYQT NAMES windeployqt HINTS ${Qt6_DIR}/../../../bin)
    if (WINDEPLOYQT)
        add_custom_command(TARGET fox_mocap POST_BUILD
            COMMAND "${WINDEPLOYQT}" --no-translations --no-compiler-runtime
                    "$<TARGET_FILE:fox_mocap>"
            COMMENT "Running windeployqt")
    endif()
endif()

message(STATUS "Fox Mocap will be built into: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "Qt6 found at:                  ${Qt6_DIR}")
