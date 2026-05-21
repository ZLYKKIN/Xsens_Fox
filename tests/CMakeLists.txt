# Unit tests for the pure logic extracted from main.cpp (foxmath / foxwire).
# Built only when -DFOX_BUILD_TESTS=ON; the Windows release build is unaffected.
# These targets need only Qt6 Core/Gui (no Widgets / OpenGL / Network / Xsens /
# Manus), so they build and run on a plain Linux CI runner.

find_package(Qt6 REQUIRED COMPONENTS Core Gui)

# The exact source compiled into the app — tests exercise the real code, not a
# copy.  Built as a tiny static lib shared by both test executables.
add_library(foxcore STATIC
    ${CMAKE_SOURCE_DIR}/scr/foxmath.cpp
    ${CMAKE_SOURCE_DIR}/scr/foxwire.cpp)
target_include_directories(foxcore PUBLIC ${CMAKE_SOURCE_DIR}/scr)
target_link_libraries(foxcore PUBLIC Qt6::Core Qt6::Gui)

add_executable(test_foxmath test_foxmath.cpp)
target_link_libraries(test_foxmath PRIVATE foxcore)
add_test(NAME foxmath COMMAND test_foxmath)

add_executable(test_foxwire test_foxwire.cpp)
target_link_libraries(test_foxwire PRIVATE foxcore)
add_test(NAME foxwire COMMAND test_foxwire)
