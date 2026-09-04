add_library(logger)

target_sources(logger
    PRIVATE
        SimulatorDataLogger.cpp
        
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_SOURCE_DIR}/include
        FILES
            ${CMAKE_SOURCE_DIR}/include/logger/SimulatorDataLogger.h
)