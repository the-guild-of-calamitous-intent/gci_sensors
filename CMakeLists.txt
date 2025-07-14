
# Specify the minimum version of CMake
cmake_minimum_required(VERSION 3.20)

# Set the Pico SDK path (adjust if needed)
set(PICO_SDK_PATH "$ENV{HOME}/github/pico-sdk")
set(PICO_BOARD "pico") # before pico_sdk_import.cmake, pico, pico2, pico_w

# Include the Pico SDK
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set project name and language
project(gci_sensors C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 23) # Options: 90, 99, 11, 17, 23 (or gnu90, gnu99, etc.)
# set(CMAKE_C_STANDARD_REQUIRED ON) # Enforce the standard
# set(CMAKE_C_EXTENSIONS OFF) # Disable GNU extensions (optional, use ON for gnuXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Os")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Os")
string(PREPEND CMAKE_EXE_LINKER_FLAGS " -Wl,--print-memory-usage ")

if(PROJECT_IS_TOP_LEVEL)
  cmake_host_system_information(RESULT HOST QUERY HOSTNAME)
  cmake_host_system_information(RESULT OSN QUERY OS_NAME)
  cmake_host_system_information(RESULT OS_VERSION QUERY OS_RELEASE)
  cmake_host_system_information(RESULT PROC QUERY PROCESSOR_DESCRIPTION)

  message(STATUS "-------------------------------------")
  message(STATUS " Project: ${PROJECT_NAME}")
  message(STATUS "-------------------------------------")
  message(STATUS " Compiler")
  message(STATUS "  C ${CMAKE_C_STANDARD}: ${CMAKE_C_COMPILER_ID} ${CMAKE_C_COMPILER_VERSION} (${CMAKE_C_COMPILER})")
  message(STATUS "  CXX ${CMAKE_CXX_STANDARD}: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION} (${CMAKE_CXX_COMPILER})")
  message(STATUS " ")
  message(STATUS " Hardware: ${PICO_BOARD} ${PICO_PLATFORM}")
  message(STATUS "-------------------------------------")
  message(STATUS " ${HOST}")
  message(STATUS " ${OSN}: ${OS_VERSION}")
  message(STATUS " ${PROC}")
  message(STATUS "-------------------------------------")

  set(BUILD_EXAMPLES ON)
else()
  if(PICO_PLATFORM)
    message(STATUS "-> ${PROJECT_NAME} is submodule for ${PICO_PLATFORM}")
  else()
    message(STATUS "-> ${PROJECT_NAME} is submodule")
  endif()
  set(BUILD_EXAMPLES OFF)
endif()

# Initialize the Pico SDK
pico_sdk_init()

LIST(APPEND pico_libs
  pico_stdlib
  hardware_i2c
  hardware_spi
)

add_library(${PROJECT_NAME}
  src/algorithms.c
  src/bmp390.c
  src/hmc5883.c
  src/io-common.c
  src/io-i2c.c
  src/io-spi.c
  src/lis3mdl.c
  src/lps22.c
  src/lsm6dsox.c
  src/pa1010d.c
)
target_include_directories(${PROJECT_NAME} 
  PUBLIC 
    src
)
target_link_libraries(${PROJECT_NAME} 
  PUBLIC 
    ${pico_libs}
)


pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)


if (BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()
