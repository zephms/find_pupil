
project(cpp000)
cmake_minimum_required(VERSION 3.2.0)
add_definitions(-std=c++11)

add_definitions(-g -Os)

# set(CMAKE_CXX_COMPILER "D:/tool/mingw810/bin/g++.exe")

# set(OpenCV_DIR D:\\tool\\opencv\\opencv455b\\install)
set(OpenCV_DIR /home/kali/dlib0/opencvb/)

find_package(OpenCV REQUIRED)


include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(cpp3 main.cpp)
# add_library(cpp3 SHARED main.cpp)

install(TARGETS cpp3 LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

# include_directories(D:\\tool\\dlib\\dlib1923install\\include)
include_directories(/home/kali/dlib0/dlibi2/include/)

set(SRC  ${PROJECT_SOURCE_DIR}/main.cpp)

link_directories(${PROJECT_SOURCE_DIR})


# link_directories("D:\tool\\dlib\\dlib1923install\\lib")
link_directories(/home/kali/dlib0/dlibi2/lib/)

set(dlib_DIR /home/kali/dlib0/dlibi2/lib/cmake/dlib/)

find_package(dlib REQUIRED)
target_link_libraries(cpp3 ${OpenCV_LIBS} dlib::dlib)
add_compile_options(-fPIC -no-pie)


