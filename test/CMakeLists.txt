cmake_minimum_required(VERSION 3.20)
project(PyTraj_test)

# Tau requires at least C11/C++11
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)

include(FetchContent)
FetchContent_Declare(
    Tau
    URL https://github.com/jasmcaus/tau/archive/dev.zip
)

FetchContent_MakeAvailable(Tau)

enable_testing()

add_executable(${PROJECT_NAME}
    atmosphere_test.c
)

target_link_libraries(
    PyTraj_test
    Tau
)