project(minionsCalibrate)
 
cmake_minimum_required(VERSION 2.6)
set(CMAKE_CXX_STANDARD 11)

find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)

include_directories( ${CMAKE_CURRENT_BINARY_DIR}  ${OpenCV_INCLUDE_DIRS} )

add_executable(minionsCalibrate main.cpp)

target_link_libraries(minionsCalibrate  ${OpenCV_LIBS} )

install(TARGETS minionsCalibrate RUNTIME DESTINATION bin)
 