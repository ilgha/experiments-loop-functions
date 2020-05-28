add_library(SDBCLoopFunc SHARED
                        RepertoireTrainingLoopFunc.cpp
                        RepertoireTrainingLoopFunc.h
                        Objects/Box.cpp
                        Objects/Box.h
                        ${LOOP_HEADERS}
                        ${LOOP_SOURCES})
target_link_libraries(SDBCLoopFunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)
