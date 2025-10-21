cmake_minimum_required(VERSION 3.10)

project(rtk_gnss_config VERSION 1.0)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -g")

# ---------------------------------------------------------
# Include directories
# ---------------------------------------------------------
include_directories(
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/rtk         # where rtklib.h + rtkcmn.c are
)

# ---------------------------------------------------------
# RTKLIB core file
# ---------------------------------------------------------
set(RTK_CORE
    rtk/rtkcmn.c
)

# ---------------------------------------------------------
# Common sources
# ---------------------------------------------------------
set(SOURCES
    src/ini.c
    src/rtk_gnss_config.c
    src/MiniUBX.cpp
    src/UBX_parser.cpp
)

# ---------------------------------------------------------
# Executables
# ---------------------------------------------------------
set(SOURCES_TEST_CONFIG
    src/main_config.c
)

set(SOURCES_TEST_SERIAL_DIAGNOSTIC
    src/main_serial_rawx_sfrbx_diagnostic.cpp
)

set(HEADERS
    src/ini.h
    src/rtk_gnss_config.h
    src/MiniUBX.hpp
    src/UBX_parser.hpp
    src/ceserial.h
)

# ---------------------------------------------------------
# Targets
# ---------------------------------------------------------
add_executable(test_config ${SOURCES} ${SOURCES_TEST_CONFIG} ${RTK_CORE} ${HEADERS})
add_executable(test_serial_diagnostic ${SOURCES} ${SOURCES_TEST_SERIAL_DIAGNOSTIC} ${RTK_CORE} ${HEADERS})

# ---------------------------------------------------------
# Output directory
# ---------------------------------------------------------
set_target_properties(test_config PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
set_target_properties(test_serial_diagnostic PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# ---------------------------------------------------------
# Link libraries (math + Windows multimedia timer)
# ---------------------------------------------------------
target_link_libraries(test_config PRIVATE m -lwinmm)
target_link_libraries(test_serial_diagnostic PRIVATE m -lwinmm)
