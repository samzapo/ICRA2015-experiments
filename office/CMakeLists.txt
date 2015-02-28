cmake_minimum_required(VERSION 2.8.3)

project(Office)

include (CheckIncludeFiles)
include (CheckLibraryExists)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/CMakeModules)

# setup include directories
include_directories(".")
set(PROJECT_SRC_DIR .)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

#find BOOST libraries
set(Boost_ADDITIONAL_VERSIONS "1.55.0" "1.56.0")
find_package(Boost COMPONENTS system REQUIRED)
link_directories(${BOOST_LIBRARY_DIRS})
include_directories(${BOOST_INCLUDE_DIR})

option(BUILD_SHARED_LIBS "Build as a shared library?" ON)

# fix the C++ linking error on 64-bit Linux
set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl")

include_directories(
                    /usr/include
                    /usr/local/include
                    )

link_directories(
                  /usr/lib
                  /usr/local/lib
                  )

add_library(office-init MODULE office-init.cpp)

include (FindPkgConfig)
IF(PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
ENDIF(PKG_CONFIG_FOUND)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

target_link_libraries(office-init Ravelin Moby ${GAZEBO_LIBRARIES} ${BOOST_LIBRARIES} ${Boost_SYSTEM_LIBRARY} protobuf )
