cmake_minimum_required(VERSION 3.5)

## Project branding, version and package mantainer
project(ros2_grsim)
set(VERSION "1.0.0a2")
#set(VENDOR "Parsian")
set(MAINTAINER "Mani Monajjemi <mani.monajjemi@gmail.com>")

# some utils and helper vars
string(TOLOWER ${CMAKE_PROJECT_NAME} CMAKE_PROJECT_NAME_LOWER)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)
include(${PROJECT_SOURCE_DIR}/cmake/Utils.cmake)
standard_config()
standard_paths(${PROJECT_SOURCE_DIR} bin lib)

set(app ${CMAKE_PROJECT_NAME})
# create the target before the sources list is known so that we can call
# add_dependencies(<target> external_proj)
# add_executable(${PROJECT_NAME}_node "")

# definitions for knowing the OS from the code
add_definitions(-DHAVE_UNIX)
#TODO: fix this, say we have FreeBSD, that's not linux
add_definitions(-DHAVE_LINUX)

# set explicitly the c++ standard to use
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Handling depenendcies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Protobuf REQUIRED)


# we will append all libs to this var
set(libs)

find_package(OpenGL REQUIRED)
find_package(ODE REQUIRED)
find_package(VarTypes)
find_package(Protobuf REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets OpenGL Network REQUIRED)

list(APPEND libs ${OPENGL_LIBRARIES})
list(APPEND libs ode::ode)
list(APPEND libs Qt5::Core Qt5::Widgets Qt5::OpenGL Qt5::Network)
list(APPEND libs ${VARTYPES_LIBRARIES})
list(APPEND libs ${PROTOBUF_LIBRARIES})

include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/net)
include_directories(${PROJECT_SOURCE_DIR}/include/physics)

protobuf_generate_cpp(PROTO_CPP PROTO_H
    src/proto/messages_robocup_ssl_detection.proto
    src/proto/messages_robocup_ssl_geometry.proto
    src/proto/messages_robocup_ssl_wrapper.proto
    src/proto/messages_robocup_ssl_refbox_log.proto
    src/proto/messages_robocup_ssl_robot_status.proto
    src/proto/grSim_Replacement.proto
    src/proto/grSim_Commands.proto
    src/proto/grSim_Packet.proto
)

qt5_add_resources(RESOURCES
    resources/textures.qrc
)

set(RESOURCES
    ${RESOURCES}
    resources/grsim.rc
)

set(SOURCES
    src/main.cpp
    src/mainwindow.cpp
    src/glwidget.cpp
    src/graphics.cpp
    src/physics/pworld.cpp
    src/physics/pobject.cpp
    src/physics/pball.cpp
    src/physics/pground.cpp
    src/physics/pfixedbox.cpp
    src/physics/pcylinder.cpp
    src/physics/pbox.cpp
    src/physics/pray.cpp
    src/net/robocup_ssl_server.cpp
    src/net/robocup_ssl_client.cpp
    src/sslworld.cpp
    src/robot.cpp
    src/configwidget.cpp
    src/statuswidget.cpp
    src/logger.cpp
    src/robotwidget.cpp
    src/getpositionwidget.cpp
)

set(HEADERS
    include/mainwindow.h
    include/glwidget.h
    include/graphics.h
    include/physics/pworld.h
    include/physics/pobject.h
    include/physics/pball.h
    include/physics/pground.h
    include/physics/pfixedbox.h
    include/physics/pcylinder.h
    include/physics/pbox.h
    include/physics/pray.h
    include/net/robocup_ssl_server.h
    include/net/robocup_ssl_client.h
    include/sslworld.h
    include/robot.h
    include/configwidget.h
    include/statuswidget.h
    include/logger.h
    include/robotwidget.h
    include/getpositionwidget.h
    include/common.h
    include/config.h
)

# files to be compiled
set(srcs
    ${CONFIG_FILES}
    ${PROTO_CPP}
    ${PROTO_H}
    ${RESOURCES}
    ${HEADERS}
    ${SOURCES}
)

file(GLOB CONFIG_FILES "config/*.ini")
set_source_files_properties(${CONFIG_FILES}  PROPERTIES MACOSX_PACKAGE_LOCATION "config")

add_executable(${PROJECT_NAME}_node ${srcs})
target_include_directories(${PROJECT_NAME}_node PRIVATE ${VARTYPES_INCLUDE_DIRS})
# target_sources(${app} PRIVATE ${srcs})
install(
  DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}_node ${libs})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES resources/grsim.desktop DESTINATION share/applications)
install(FILES resources/icons/grsim.svg DESTINATION share/icons/hicolor/scalable/apps)

option(BUILD_CLIENTS "Choose this option if you want to build the example Qt client." ON)
if(BUILD_CLIENTS)
    add_subdirectory(clients/qt)
endif()

file(COPY README.md LICENSE.md DESTINATION ${CMAKE_BINARY_DIR})
file(RENAME ${CMAKE_BINARY_DIR}/README.md ${CMAKE_BINARY_DIR}/README.txt)
file(RENAME ${CMAKE_BINARY_DIR}/LICENSE.md ${CMAKE_BINARY_DIR}/LICENSE.txt)

ament_package()