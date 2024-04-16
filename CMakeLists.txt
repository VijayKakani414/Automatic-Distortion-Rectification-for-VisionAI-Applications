cmake_minimum_required(VERSION 3.5)
project(cpp_distortion_rectify VERSION 1.0.0)

# Configure the global output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/dist)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/dist)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/dist)

# configure OpenCV
find_package(OpenCV REQUIRED)

# Add projects here
add_subdirectory(libdistrect)
add_subdirectory(sample)


# Copy the external dependencies
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/libs/EDPFLinesmex.mexw64 DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/dist/Debug)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/libs/EDPFLinesmex.mexw64 DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/dist/Release)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/libs/opencv_core246.dll DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/dist/Debug)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/libs/opencv_core246.dll DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/dist/Release)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/libs/opencv_imgproc246.dll DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/dist/Debug)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/libs/opencv_imgproc246.dll DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/dist/Release)