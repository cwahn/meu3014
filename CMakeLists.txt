cmake_minimum_required(VERSION 3.5.0)
project(2nd VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

include(CTest)
enable_testing()

include(FetchContent)

find_package(efp QUIET)

if(NOT efp_FOUND)
    FetchContent_Declare(
        efp
        GIT_REPOSITORY http://github.com/cwahn/efp
        GIT_TAG main
    )
    FetchContent_MakeAvailable(efp)
endif()

if(NOT efp_logger_FOUND)
    FetchContent_Declare(
        efp_logger
        GIT_REPOSITORY http://github.com/cwahn/efp-logger
        GIT_TAG main
    )
    FetchContent_MakeAvailable(efp_logger)
endif()

# find_library(pigpio_lib pigpio)
# if(NOT pigpio_lib)
#   message(FATAL_ERROR "pigpio library not found")
# endif()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

add_subdirectory(app)