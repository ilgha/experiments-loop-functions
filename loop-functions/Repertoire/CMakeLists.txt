add_library(SDBCLoopFunc SHARED
                        SDBCLoopFunc.cpp
                        SDBCLoopFunc.h
                        QualityMetricCollision.cpp
                        QualityMetricCollision.h
                        ${LOOP_HEADERS}
                        ${LOOP_SOURCES})
target_link_libraries(SDBCLoopFunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)
