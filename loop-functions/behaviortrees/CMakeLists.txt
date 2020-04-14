# Set up ARGoS compilation information
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})
link_directories(
	${CMAKE_BINARY_DIR}/src
    )

add_library(naggregation_loopfunc SHARED NAggregationLoopFunc.h NAggregationLoopFunc.cpp)
target_link_libraries(naggregation_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)
