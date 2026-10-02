# Host unit tests for N2V8. Build:
#   cmake -S test/host -B build/host && cmake --build build/host -j && ctest --test-dir build/host
cmake_minimum_required(VERSION 3.20)
project(n2_host_tests CXX)

set(CMAKE_CXX_STANDARD 17)          # the R4 core builds with gnu++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
FetchContent_Declare(Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG v3.16.0
  GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(Catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)

set(N2_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

# Everything under src/. HalArduino.cpp is guarded by #ifdef ARDUINO, so it compiles to nothing here.
file(GLOB_RECURSE N2_SOURCES CONFIGURE_DEPENDS ${N2_SRC}/*.cpp)
add_library(n2_core STATIC ${N2_SOURCES})
target_include_directories(n2_core PUBLIC ${N2_SRC})
target_compile_definitions(n2_core PUBLIC N2_BUILD_HOST)
target_compile_options(n2_core PUBLIC -Wall -Wextra -Wpedantic -Werror)

file(GLOB N2_TESTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp)
add_executable(n2_tests ${N2_TESTS})
target_include_directories(n2_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(n2_tests PRIVATE n2_core Catch2::Catch2WithMain)

include(CTest)
include(Catch)
catch_discover_tests(n2_tests)

# Prints the screens for review: build/host/render_screens
add_executable(render_screens render_screens.cpp)
target_link_libraries(render_screens PRIVATE n2_core)
