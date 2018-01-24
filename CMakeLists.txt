cmake_minimum_required(VERSION 3.0)

project(grips)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_PREFIX_PATH /opt/Qt5.7.0/5.7/gcc_64;${CMAKE_PREFIX_PATH})

set(Qt5Core_DIR "/opt/Qt5.7.0/5.7/gcc_64/lib/cmake/Qt5Core")
set(Qt5_DIR "/opt/Qt5.7.0/5.7/gcc_64/lib/cmake/Qt5")
set(QT_QMAKE_EXECUTABLE "/opt/Qt5.7.0/5.7/gcc_64/bin/qmake.exe")

# add the binary tree to the search path for include files
# so that we will find GripsConfig.h
include_directories("${PROJECT_BINARY_DIR}")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(ompl REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets Charts Svg)
find_library(Qt5 HINTS /opt/Qt5.7.0/5.7/gcc_64/lib)
include_directories( ${CMAKE_BINARY_DIR} )
set(CMAKE_AUTOMOC ON)
file(GLOB GUI_SRC gui/*.?pp)
set(EXTRA_SRC ${EXTRA_SRC} ${GUI_SRC})
set(EXTRA_LIB ${EXTRA_LIB} Qt5::Core Qt5::Svg Qt5::Widgets Qt5::Charts)

option(USE_G1 "Use G1 clothoid steering (not Open Source)" OFF)
if (USE_G1)
    file(GLOB_RECURSE G1_SRC steer_functions/G1Clothoid/*.?pp)
    add_library(g1clothoid GLOB_RECURSE
            steer_functions/G1Clothoid/*.cc)
    set(EXTRA_SRC ${EXTRA_SRC} ${G1_SRC})
    set(EXTRA_LIB ${EXTRA_LIB} g1clothoid)
endif (USE_G1)


include_directories(${PROJECT_SOURCE_DIR})

file(GLOB Base base/*)
file(GLOB Utils utils/*)
file(GLOB_RECURSE Steer steer_functions/*)
file(GLOB Metrics metrics/*)
file(GLOB Planners planners/*)
set(ALL_SRC ${Base} ${Utils} ${Steer} ${Metrics} ${Planners} ${EXTRA_SRC} PostSmoothing.cpp)

link_directories(base/)
link_directories(steer_functions/)

add_executable(homotopy_test mainHomotopy.cpp ${ALL_SRC})
add_executable(thetastar_steer main.cpp ${ALL_SRC})
add_executable(shortening_test mainShortening.cpp ${ALL_SRC})
add_executable(showcase mainShowcase.cpp ${ALL_SRC})

target_link_libraries(thetastar_steer
        ${OMPL_LIBRARIES} ${OMPLAPP_LIBRARIES} ${EXTRA_LIB})

target_link_libraries(homotopy_test
        ${OMPL_LIBRARIES} ${OMPLAPP_LIBRARIES} ${EXTRA_LIB})

target_link_libraries(shortening_test
        ${OMPL_LIBRARIES} ${OMPLAPP_LIBRARIES} ${EXTRA_LIB})

target_link_libraries(showcase
        ${OMPL_LIBRARIES} ${OMPLAPP_LIBRARIES} ${EXTRA_LIB})
