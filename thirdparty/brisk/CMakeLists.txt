cmake_minimum_required(VERSION 2.4.6)
SET(PROJECT_NAME brisk)
PROJECT(${PROJECT_NAME})
#set dependancies
FIND_PACKAGE(agast REQUIRED)

file(GLOB BRISK_SOURCE_FILES  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc")
file(GLOB BRISK_HEADER_FILES  "${CMAKE_CURRENT_SOURCE_DIR}/include/brisk/*.h")

MESSAGE(${agast_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(
	${CMAKE_CURRENT_SOURCE_DIR}/include/brisk/
	${agast_INCLUDE_DIRS})

#build the library dynamic and static versions
add_library(brisk SHARED ${BRISK_SOURCE_FILES} ${BRISK_HEADER_FILES})
