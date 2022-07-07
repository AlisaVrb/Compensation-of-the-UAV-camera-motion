cmake_minimum_required(VERSION 3.19.0)

project(vehicle-classification)

cmake_policy(SET CMP0016 NEW)
cmake_policy(SET CMP0057 NEW)

set(CMAKE_CXX_STANDARD 17)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")
SET(OPENCV_MIN_VERSION "4.2")

if(MSVC)

  if(MSVC_VERSION VERSION_LESS 1920)
    message(FATAL_ERROR "${PROJECT_NAME} requires VS2015 or greater.")
  endif()

elseif(CMAKE_COMPILER_IS_GNUCXX)

  set(CXX_COMPILER_VERSION ${CMAKE_CXX_COMPILER_VERSION})
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7.5)
    message(FATAL_ERROR "The installed g++ version is ${CMAKE_CXX_COMPILER_VERSION}. ${PROJECT_NAME} requires g++ 5.3.1 or greater.")
  endif()
endif()


list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(OpenCV ${OPENCV_MIN_VERSION} REQUIRED COMPONENTS core imgproc dnn imgcodecs)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

include_directories(./include)

add_library(olvia_classifier SHARED src/vehicle_classifier.cc)
target_link_libraries(olvia_classifier ${OpenCV_LIBS})

add_executable(classifier_test src/vehicle_classifier_main.cc)
target_link_libraries(classifier_test ${OpenCV_LIBS} olvia_classifier)

install(TARGETS olvia_classifier RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(TARGETS classifier_test  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
