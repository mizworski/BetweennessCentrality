cmake_minimum_required(VERSION 3.6)
project(pw2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-Wpedantic -Werror -Wall -Wextra")
set(CMAKE_CXX_FLAGS "-pthread -O0")
SET (CMAKE_CXX_COMPILER "/usr/bin/clang++")
set(SOURCE_FILES brandes.cc worker.cc worker.h brandes.h)
add_executable(brandes ${SOURCE_FILES})