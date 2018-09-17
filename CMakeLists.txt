cmake_minimum_required(VERSION 2.8.3)
project(lpf_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        dynamic_reconfigure
        roscpp)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
        cfg/Lpf.cfg
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES lpf_ros
        #  CATKIN_DEPENDS other_catkin_pkg
        # DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

set(
        PROJECT_FILES
        src/lpf_ros_node.cpp
        src/lpf_ros.cpp
        include/${PROJECT_NAME}/lpf_ros.h
)

## Declare a C++ library
add_library(
        ${PROJECT_NAME}
        ${PROJECT_FILES}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(
        ${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(
        ${PROJECT_NAME}_node
        ${PROJECT_FILES}
)



# make sure configure headers are built before any node using them
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )