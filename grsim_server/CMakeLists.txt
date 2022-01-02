cmake_minimum_required(VERSION 3.5)

project(grsim_server)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)
set(CMAKE_AUTOMOC YES)
set(CMAKE_INCLUDE_CURRENT_DIR YES)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grsim_msgs REQUIRED)

find_package(Protobuf REQUIRED)
find_package(OpenGL REQUIRED)
find_package(ODE REQUIRED)
find_package(VarTypes REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets OpenGL Network REQUIRED)

set(libs)
list(APPEND libs ode::ode)
list(APPEND libs Qt5::Core Qt5::Widgets Qt5::OpenGL Qt5::Network)
list(APPEND libs ${OPENGL_LIBRARIES} ${VARTYPES_LIBRARIES} ${PROTOBUF_LIBRARIES})

include_directories(${OPENGL_INCLUDE_DIR} ${PROTOBUF_INCLUDE_DIRS} ${VARTYPES_INCLUDE_DIRS})
include_directories(include include/net include/physics include/ros)

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
    include/ros/robocup_ssl_server_component.h
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

set(srcs
    ${PROTO_CPP}
    ${PROTO_H}
    ${HEADERS}
    ${SOURCES}
    ${RESOURCES}
)

file(GLOB CONFIG_FILES "config/*.ini")
set_source_files_properties(${CONFIG_FILES}  PROPERTIES MACOSX_PACKAGE_LOCATION "config")

add_executable(${PROJECT_NAME}_node ${srcs})

ament_target_dependencies(
  ${PROJECT_NAME}_node
  "rclcpp"
  "grsim_msgs"
)

install(
  DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}_node ${libs})

install(DIRECTORY   include/config
         DESTINATION share/${PROJECT_NAME})

install(FILES       resources/grsim.desktop
        DESTINATION share/applications)
install(FILES       resources/icons/grsim.svg 
        DESTINATION share/icons/hicolor/scalable/apps)

ament_package()