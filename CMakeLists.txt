cmake_minimum_required(VERSION 3.5.0)
project(2nd VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

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

# fmt
find_package(fmt QUIET)

if(NOT fmt_FOUND)
    FetchContent_Declare(
        fmt
        GIT_REPOSITORY https://github.com/fmtlib/fmt.git
        GIT_TAG master
    )
    FetchContent_MakeAvailable(fmt)
endif()

file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "include/*.hpp")
add_library(meu3014_hal ${SOURCES} ${HEADERS})
target_link_libraries(meu3014_hal PRIVATE efp efp_logger fmt)
target_include_directories(meu3014_hal PUBLIC include)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

add_subdirectory(app)
