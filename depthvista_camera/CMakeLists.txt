cmake_minimum_required(VERSION 3.5)

project(depthvista_camera VERSION 1.0.1 DESCRIPTION "DepthVista ROS Wrapper")

add_compile_options(-std=c++11)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosconsole
    sensor_msgs
	std_msgs
	nodelet
    cv_bridge
    image_transport
    dynamic_reconfigure
)

function(checkPackage package customMessage)
    set(varName "${package}_FOUND")
    if (NOT "${${varName}}")
        string(REPLACE "_" "-" aptPackage ${package})
        if("${customMessage}" STREQUAL "")
            message(FATAL_ERROR "\n\n ${package} is missing, please try to install it with:\n sudo apt-get install ros-$(rosversion -d)-${aptPackage}\n\n")
        else()
            message(FATAL_ERROR "\n\n ${customMessage} \n\n")
        endif()
    endif()
endfunction(checkPackage)

find_package(depthvista REQUIRED)
checkPackage("depthvista" "libdepthvista not found, install it")
find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
	cfg/DepthVista.cfg
)

catkin_package(
  	LIBRARIES ${PROJECT_NAME}
  	CATKIN_DEPENDS message_runtime roscpp sensor_msgs std_msgs
	nodelet
	cv_bridge
    image_transport
    dynamic_reconfigure
)

set(DEPTHVISTA_NODELET_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/depthvista_nodelet.cpp)

set(INCLUDE_DIRS
    ${catkin_INCLUDE_DIRS}
    ${DEPTHVISTA_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(DEPTHVISTA_LIBRARIES depthvista)

add_definitions(-std=c++14 -Wno-deprecated-declarations)
set(LINK_LIBRARIES
  ${catkin_LIBRARIES}
  ${DEPTHVISTA_LIBRARIES}
  stdc++fs
)

add_library(depthvista_camera
    ${DEPTHVISTA_NODELET_SRC}
)

target_include_directories(depthvista_camera PRIVATE ${INCLUDE_DIRS})
target_link_libraries(depthvista_camera ${LINK_LIBRARIES})
add_dependencies(
    depthvista_camera
    ${catkin_EXPORTED_TARGETS}
	${PROJECT_NAME}_gencfg
)

FILE(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_files_${PROJECT_NAME} SOURCES ${all_files})

install(TARGETS
  depthvista_camera
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES
  nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
