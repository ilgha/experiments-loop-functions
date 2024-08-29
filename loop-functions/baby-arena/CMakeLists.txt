# Set up ARGoS compilation information
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})
link_directories(
	${CMAKE_BINARY_DIR}/src
    )

add_library(baby_foraging_loopfunc SHARED BabyForagingTwoSpotsLoopFunc.h BabyForagingTwoSpotsLoopFunc.cpp)
target_link_libraries(baby_foraging_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(chocolate_baby_aac_loopfunc SHARED ChocolateBabyAACLoopFunc.h ChocolateBabyAACLoopFunc.cpp ${LOOP_HEADERS} ${LOOP_SOURCES})
target_link_libraries(chocolate_baby_aac_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(baby_grid_exploration_loopfunc SHARED BabyGridExplorationLoopFunc.h BabyGridExplorationLoopFunc.cpp ${LOOP_HEADERS} ${LOOP_SOURCES})
target_link_libraries(baby_grid_exploration_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)
