cmake_minimum_required(VERSION 3.17)

set(SOURCES
        src/ADC.cpp
        src/Channel.cpp
)

set(COMPONENTS
        esp_adc
)

idf_component_register(SRCS ${SOURCES} INCLUDE_DIRS include REQUIRES ${COMPONENTS})
