# Copyright: (C) 2017 Social Robotics Lab - Yale University
# Author: Alessandro Roncone <alessandro.roncone@yale.edu>
# CopyPolicy: Released under the terms of the LGPLv2.1.

cmake_minimum_required(VERSION 2.8.3)

# find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
# find_package(OpenCV REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)

## Declare the C++ libraries
add_library(phasespace_publisher  include/phasespace_publisher/phasespace_publisher.h
                                  src/phasespace_publisher/phasespace_publisher.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(phasespace_publisher ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(phasespace_publisher   ${catkin_LIBRARIES})

## Mark libraries for installation
install (TARGETS phasespace_publisher
         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY   ${PROJECT_SOURCE_DIR}/include/phasespace_publisher
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)
