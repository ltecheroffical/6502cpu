cmake_minimum_required(VERSION 3.16)
project(6502emu VERSION 1.0.0 LANGUAGES C)

file(GLOB_RECURSE C_SOURCE_FILES CONFIGURE_DEPENDS 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)

add_library(
    ${PROJECT_NAME}

    ${C_SOURCE_FILES}
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/lib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src)
