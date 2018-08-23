cmake_minimum_required(VERSION 2.8)
project ( mycam )

set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

#find Threads
find_package (Threads REQUIRED)
if (THREADS_FOUND)
    message ("-- Threads found")
    set (LIBS ${LIBS} "Threads::Threads")
endif (THREADS_FOUND)

find_package (OpenCV REQUIRED)
if (OpenCV_FOUND)
    message ("-- OpenCV found")
    set (LIBS ${LIBS} ${OpenCV_LIBRARIES} )
endif (OpenCV_FOUND)

# include Dlib library
include ( /usr/include/dlib/cmake )

include_directories ( ${PROJECT_SOURCE_DIR}/Core
                      ${PROJECT_SOURCE_DIR}/Utils
                      ${PROJECT_SOURCE_DIR}/Detection
                      ${PROJECT_SOURCE_DIR}/Tracking)

file (GLOB CORE_SOURCES "${PROJECT_SOURCE_DIR}/Core/*.cpp")
file (GLOB UTILS_SOURCES "${PROJECT_SOURCE_DIR}/Utils/*.cpp")
file (GLOB DETECTION_SOURCES "${PROJECT_SOURCE_DIR}/Detection/*.cpp")
file (GLOB TRACKER_SOURCES "${PROJECT_SOURCE_DIR}/Tracking/*.cpp")
add_executable(mycam ${CORE_SOURCES} ${UTILS_SOURCES} ${DETECTION_SOURCES} ${TRACKER_SOURCES})

target_link_libraries ( mycam dlib::dlib ${LIBS} ) 
