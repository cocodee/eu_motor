cmake_minimum_required(VERSION 3.10.0)

project(eu_motor_project)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED EYOU_LIB_PATH)
    set(EYOU_LIB_PATH ${CMAKE_SOURCE_DIR}/../lib)
endif()
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${EYOU_LIB_PATH})

add_library(eu_motor eu_motor.cpp)
add_executable(test_suite example/test_suite.cpp)

if(WIN32)
    target_link_libraries(eu_motor ${EYOU_LIB_PATH}/eu_harmonic.dll)
elseif(UNIX)
    target_link_libraries(eu_motor ${EYOU_LIB_PATH}/libeu_harmonic.so pthread)
    target_link_libraries(test_suite ${EYOU_LIB_PATH}/libeu_harmonic.so pthread eu_motor)
endif()
