cmake_minimum_required(VERSION 3.4)
project(openmesh-local-rivara)
option(USE_SYSTEM_OPENMESH "" OFF)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    message(STATUS "Using default BUILD_TYPE Release.")
endif ()

#
# OpenMesh
#
if (USE_SYSTEM_OPENMESH)
    find_package(OpenMesh REQUIRED)
    include_directories(SYSTEM ${OPENMESH_INCLUDE_DIR})
else ()
    set(OPENMESH "${CMAKE_CURRENT_BINARY_DIR}/OPENMESH")
    externalproject_add(openmesh_project
            GIT_REPOSITORY https://www.graphics.rwth-aachen.de:9000/OpenMesh/OpenMesh.git
            GIT_TAG OpenMesh-6.3
            PREFIX "${OPENMESH}"
            UPDATE_COMMAND ""
            CMAKE_ARGS
            "-G${CMAKE_GENERATOR}"
            "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
            "-DCMAKE_INSTALL_PREFIX=${OPENMESH}"
            "-DBUILD_APPS=OFF"
            )

    add_library(openmesh_core SHARED IMPORTED)
    add_library(openmesh_tools SHARED IMPORTED)
    if (CMAKE_BUILD_TYPE MATCHES Debug)
        message(STATUS "Using OpenMesh debug libraries.")
        set_target_properties(openmesh_core PROPERTIES IMPORTED_LOCATION ${OPENMESH}/lib/libOpenMeshCored.so)
        set_target_properties(openmesh_tools PROPERTIES IMPORTED_LOCATION ${OPENMESH}/lib/libOpenMeshToolsd.so)
    else ()
        set_target_properties(openmesh_core PROPERTIES IMPORTED_LOCATION ${OPENMESH}/lib/libOpenMeshCore.so)
        set_target_properties(openmesh_tools PROPERTIES IMPORTED_LOCATION ${OPENMESH}/lib/libOpenMeshTools.so)
    endif ()

    add_dependencies(openmesh_core openmesh_project)
    add_dependencies(openmesh_tools openmesh_project)
    include_directories(SYSTEM ${OPENMESH}/include)
    set(OPENMESH_LIBRARIES openmesh_core openmesh_tools)
endif ()

set(SOURCE_FILES main.cpp)
add_executable(openmesh-local-rivara ${SOURCE_FILES})
target_link_libraries(openmesh-local-rivara ${CMAKE_DL_LIBS} ${OPENMESH_LIBRARIES})

