project(sample VERSION 1.0.0)

set(SAMPLE_SRC main.cpp)

include_directories(${LIBDISTRECT_INC_DIRS})
add_definitions(${LIBDISTRECT_CXX_DEFS})
link_directories(${LIBDISTRECT_LIB_DIRS})

add_executable(sample ${SAMPLE_SRC})
target_link_libraries(sample libdistrect ${LIBDISTRECT_LIBS})