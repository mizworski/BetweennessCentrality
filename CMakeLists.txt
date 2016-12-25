cmake_minimum_required(VERSION 3.6)
project(pw2)

set(CMAKE_CXX_STANDARD 14)

set(SOURCE_FILES brandes.cc)
add_executable(brandes ${SOURCE_FILES})