cmake_minimum_required(VERSION 3.10)
project(path_planner)

set(CMAKE_CXX_STANDARD 20)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GDAL REQUIRED gdal)

find_package(ompl REQUIRED)

include_directories(${GDAL_INCLUDE_DIRS})
link_directories(${GDAL_LIBRARY_DIRS})

add_executable(path_planner
    main.cpp
    src/DEMMap.cpp
    src/ConstraintChecker.cpp
        src/WaypointPath.h
)

target_include_directories(path_planner PRIVATE src)
include_directories(${OMPL_INCLUDE_DIRS})
target_link_libraries(path_planner PRIVATE ${OMPL_LIBRARIES} ${GDAL_LIBRARIES})
#target_link_libraries(path_planner
#
#    ompl
#)
