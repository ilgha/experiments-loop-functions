# Set up ARGoS compilation information
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})
link_directories(
	${CMAKE_BINARY_DIR}/src
    )

add_library(template SHARED Template.h Template.cpp)
target_link_libraries(template argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(aac SHARED AggregationAmbientCues.h AggregationAmbientCues.cpp)
target_link_libraries(aac argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(cfa SHARED CFA.h CFA.cpp ${LOOP_HEADERS} ${LOOP_SOURCES})
target_link_libraries(cfa argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(homing SHARED Homing.h Homing.cpp ${LOOP_HEADERS} ${LOOP_SOURCES})
target_link_libraries(homing argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(sca SHARED SCA.h SCA.cpp ${LOOP_HEADERS} ${LOOP_SOURCES})
target_link_libraries(sca argos3plugin_${ARGOS_BUILD_FOR}_epuck)
