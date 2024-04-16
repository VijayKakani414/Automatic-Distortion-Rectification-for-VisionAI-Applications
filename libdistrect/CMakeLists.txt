project(libdistrect VERSION 1.0.0)

include(Matlab.cmake)
include(GetFMin.cmake)

set(LIBDISTRECT_INC_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${MATLAB_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    CACHE PATH "Set the include directories for Matlab"
)
set(LIBDISTRECT_CXX_DEFS 
    ${MATLAB_CXX_DEFS}
    CACHE STRING "CXX compiler flags"
)
set(LIBDISTRECT_LIB_DIRS
    ${MATLAB_LIB_DIRS}
    CACHE PATH "Set Matlab library path"
)
set(LIBDISTRECT_LIBS
    ${MATLAB_LIBS}
    ${OpenCV_LIBS}
    CACHE INTERNAL ""
)

include_directories(${LIBDISTRECT_INC_DIRS})
add_definitions(${LIBDISTRECT_CXX_DEFS})
link_directories(${LIBDISTRECT_LIB_DIRS})

set(LIBDISTRECT_SRC_FILES
    include/libdistrect.hpp
    src/libdistrect.cpp
    src/ilinesegment.cpp
)

add_library(libdistrect ${LIBDISTRECT_SRC_FILES})
target_link_libraries(libdistrect ${LIBDISTRECT_LIBS})
