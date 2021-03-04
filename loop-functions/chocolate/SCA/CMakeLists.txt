add_library(chocolate_sca_loopfunc SHARED ChocolateSCALoopFunc.h ChocolateSCALoopFunc.cpp ${LOOP_HEADERS} ${LOOP_SOURCES})
target_link_libraries(chocolate_sca_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)
add_library(chocolate_sca_log_loopfunc SHARED ChocolateSCALogLoopFunc.h ChocolateSCALogLoopFunc.cpp ${LOOP_HEADERS} ${LOOP_SOURCES})
target_link_libraries(chocolate_sca_log_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)
