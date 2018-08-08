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

include ( /usr/include/dlib/cmake )
include_directories(include)
file (GLOB SOURCES "src/*.cpp")
add_executable(mycam ${SOURCES})

target_link_libraries ( mycam dlib::dlib ${LIBS} ) 
