cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME twitch_gimbal)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    RM2023_Lib_V1.2/communication.c
    RM2023_Lib_V1.2/CP_System.c
    RM2023_Lib_V1.2/RM_Lib.cpp
    CH010/example_data.c
    CH010/hipnuc_dec.c
    CH010/nmea_dec.c
    Math_Lib/PID.cpp
    Math_Lib/SMC.cpp
    Motor_Lib/DM.cpp
    Motor_Lib/RM.cpp
    Motor_Lib/RMD.cpp
    Control/Pitch.cpp
    Control/Shoot.cpp
    Control/Yaw.cpp
    App_Lib/app_gimbal.cpp
    App_Lib/main.cpp
)

# These .c files include C++ application/library headers.
set_source_files_properties(
    Core/Src/stm32f4xx_it.c
    RM2023_Lib_V1.2/communication.c
    PROPERTIES LANGUAGE CXX
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Control
    Math_Lib
    Motor_Lib
    RM2023_Lib_V1.2
    CH010
    App_Lib
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    TWITCH_GIMBAL_USE_CPP_MAIN
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
