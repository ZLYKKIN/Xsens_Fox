# Unit tests for the pure logic extracted from main.cpp (foxmath / foxwire).
# Built only when -DFOX_BUILD_TESTS=ON; the Windows release build is unaffected.
# These targets need only Qt6 Core/Gui (no Widgets / OpenGL / Network / Xsens /
# Manus), so they build and run on a plain Linux CI runner.

find_package(Qt6 REQUIRED COMPONENTS Core Gui)

# Strict warnings on the pure-logic core + tests.  -Werror is opt-in (enabled in
# CI) so that a newer host compiler surfacing a fresh warning never bricks a
# local build.  ASan/UBSan is opt-in for a second, instrumented test pass.
option(FOX_WERROR        "Treat warnings as errors in the test build" OFF)
option(FOX_TEST_SANITIZE "Build the unit tests with ASan + UBSan"     OFF)

set(_fox_warn "")
set(_fox_san  "")
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(_fox_warn -Wall -Wextra)
    if(FOX_WERROR)
        list(APPEND _fox_warn -Werror)
    endif()
    if(FOX_TEST_SANITIZE)
        set(_fox_san -fsanitize=address,undefined -fno-omit-frame-pointer)
    endif()
endif()

# The exact source compiled into the app — tests exercise the real code, not a
# copy.  Built as a tiny static lib shared by both test executables.
add_library(foxcore STATIC
    ${CMAKE_SOURCE_DIR}/scr/foxmath.cpp
    ${CMAKE_SOURCE_DIR}/scr/foxwire.cpp)
target_include_directories(foxcore PUBLIC ${CMAKE_SOURCE_DIR}/scr)
target_link_libraries(foxcore PUBLIC Qt6::Core Qt6::Gui)
target_compile_options(foxcore PRIVATE ${_fox_warn} ${_fox_san})
target_link_options(foxcore PUBLIC ${_fox_san})

add_executable(test_foxmath test_foxmath.cpp)
target_link_libraries(test_foxmath PRIVATE foxcore)
target_compile_options(test_foxmath PRIVATE ${_fox_warn} ${_fox_san})
add_test(NAME foxmath COMMAND test_foxmath)

add_executable(test_foxwire test_foxwire.cpp)
target_link_libraries(test_foxwire PRIVATE foxcore)
target_compile_options(test_foxwire PRIVATE ${_fox_warn} ${_fox_san})
add_test(NAME foxwire COMMAND test_foxwire)
