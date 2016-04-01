cmake_minimum_required(VERSION 3.4)
project(openmesh-local-rivara)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

find_package(OpenMesh REQUIRED)
include_directories(${OPENMESH_INCLUDE_DIR})

set(SOURCE_FILES main.cpp)
add_executable(openmesh-local-rivara ${SOURCE_FILES})
target_link_libraries(openmesh-local-rivara ${CMAKE_DL_LIBS} ${OPENMESH_LIBRARIES})

