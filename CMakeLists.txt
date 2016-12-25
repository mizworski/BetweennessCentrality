cmake_minimum_required(VERSION 3.6)
project(pw2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-Wpedantic -Werror -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O0")

set(SOURCE_FILES brandes.cc)
add_executable(brandes ${SOURCE_FILES})