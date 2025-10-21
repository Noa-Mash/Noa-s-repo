cmake_minimum_required(VERSION 3.10)

# ============================================================================
# Project Info
# ----------------------------------------------------------------------------
project(rtk_gnss VERSION 1.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -g")

# ============================================================================
# Include directories
# ----------------------------------------------------------------------------
include_directories(
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/rtk
    ${CMAKE_SOURCE_DIR}/rtk/rcv
)

# ============================================================================
# RTKLIB CORE SOURCES (minimal set)
# ----------------------------------------------------------------------------
# These are the only RTKLIB files you need to parse u-blox RAWX/SFRBX messages
# and extract satellite and ephemeris data.
# ----------------------------------------------------------------------------
set(RTK_CORE
    rtk/rtkcmn.c        # Common math/time utilities
    rtk/ephemeris.c     # Ephemeris decoding and satellite position
    rtk/preceph.c       # Precise ephemerides (needed for peph2pos)
    rtk/sbas.c          # SBAS corrections (for sbssatcorr reference)
    rtk/rcvraw.c        # Receiver raw data interface
    rtk/rcv/ublox.c     # u-blox driver (input_ubx, input_ubxf, etc.)
)

# ============================================================================
# Project sources
# ----------------------------------------------------------------------------
set(SOURCES
    src/ini.c
    src/rtk_gnss_config.c
    src/MiniUBX.cpp
    src/updated_UBX_parser.cpp
    src/main_serial_rawx_sfrbx_diagnostic.cpp
)

# ============================================================================
# Build target
# ----------------------------------------------------------------------------
add_executable(main_serial_rawx_sfrbx_diagnostic
    ${SOURCES}
    ${RTK_CORE}
)

# Link math + Windows multimedia libraries (for timeGetTime etc.)
target_link_libraries(main_serial_rawx_sfrbx_diagnostic
    m
    winmm
)

# ============================================================================
# Output path
# ----------------------------------------------------------------------------
set_target_properties(main_serial_rawx_sfrbx_diagnostic PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
