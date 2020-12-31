cmake_minimum_required(VERSION 3.5)
project(ros2_grsim_protobuf)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(${PROTOBUF_INCLUDE_DIRS})

protobuf_generate_cpp(PROTO_CPP PROTO_H
    proto/messages_robocup_ssl_detection.proto
    proto/messages_robocup_ssl_geometry.proto
    proto/messages_robocup_ssl_wrapper.proto
    proto/messages_robocup_ssl_refbox_log.proto
    proto/messages_robocup_ssl_robot_status.proto
    proto/grSim_Replacement.proto
    proto/grSim_Commands.proto
    proto/grSim_Packet.proto
)

install(
  FILES ${PROTO_H}
  DESTINATION include/${PROJECT_NAME})
ament_auto_add_library(${PROJECT_NAME} SHARED
 ${PROTO_CPP} ${PROTO_H})
target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()