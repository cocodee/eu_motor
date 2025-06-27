cmake_minimum_required(VERSION 3.10.0)

project(eu_motor_project)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if(NOT DEFINED EYOU_LIB_PATH)
    set(EYOU_LIB_PATH ${CMAKE_SOURCE_DIR}/../lib)
endif()

add_library(eu_motor SHARED eu_motor.cpp)
add_executable(test_suite test_suite.cpp)

if(WIN32)
    target_link_libraries(eu_motor ${EYOU_LIB_PATH}/eu_harmonic.dll)
elseif(UNIX)
    target_link_libraries(eu_motor ${EYOU_LIB_PATH}/libeu_harmonic.so pthread)
    target_link_libraries(test_suite ${EYOU_LIB_PATH}/libeu_harmonic.so pthread eu_motor)
endif()
