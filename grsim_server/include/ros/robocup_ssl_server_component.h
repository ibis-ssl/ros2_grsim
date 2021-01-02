//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
  \file    robocup_ssl_server.h
  \brief   C++ Interface: robocup_ssl_server
  \author  Stefan Zickler, 2009
  \author  Jan Segre, 2012
*/
//========================================================================
#ifndef ROBOCUP_SSL_SERVER_COMPONENT_H
#define ROBOCUP_SSL_SERVER_COMPONENT_H

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ROBOCUP_SSL_SERVER_COMPONENT_EXPORT __attribute__((dllexport))
#define ROBOCUP_SSL_SERVER_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define ROBOCUP_SSL_SERVER_COMPONENT_EXPORT __declspec(dllexport)
#define ROBOCUP_SSL_SERVER_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef ROBOCUP_SSL_SERVER_COMPONENT_BUILDING_DLL
#define ROBOCUP_SSL_SERVER_COMPONENT_PUBLIC \
    ROBOCUP_SSL_SERVER_COMPONENT_EXPORT
#else
#define ROBOCUP_SSL_SERVER_COMPONENT_PUBLIC \
    ROBOCUP_SSL_SERVER_COMPONENT_IMPORT
#endif
#define ROBOCUP_SSL_SERVER_COMPONENT_PUBLIC_TYPE \
    ROBOCUP_SSL_SERVER_COMPONENT_PUBLIC
#define ROBOCUP_SSL_SERVER_COMPONENT_LOCAL
#else
#define ROBOCUP_SSL_SERVER_COMPONENT_EXPORT \
    __attribute__((visibility("default")))
#define ROBOCUP_SSL_SERVER_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define ROBOCUP_SSL_SERVER_COMPONENT_PUBLIC \
    __attribute__((visibility("default")))
#define ROBOCUP_SSL_SERVER_COMPONENT_LOCAL \
    __attribute__((visibility("hidden")))
#else
#define ROBOCUP_SSL_SERVER_COMPONENT_PUBLIC
#define ROBOCUP_SSL_SERVER_COMPONENT_LOCAL
#endif
#define ROBOCUP_SSL_SERVER_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}// extern "C"
#endif

#include <QMutex>
#include <QObject>
#include <string>
// #include <rclcpp/rclcpp.hpp>
#include "grsim_msgs/msg/vision_detections.hpp"
#include "grsim_msgs/msg/vision_geometry.hpp"
#include "messages_robocup_ssl_wrapper.pb.h"
#include "rclcpp/rclcpp.hpp"
using namespace std;

static const double TO_METER = 0.001;

class FormatConverter {
public:
    FormatConverter() {}

    void publishSSLPacket(SSL_WrapperPacket packet) {
        if (packet.has_detection()) {
            grsim_msgs::msg::VisionDetections vision_detections;
            auto detection = constructDetectionFrame(packet.detection());
            rclcpp::Clock ros_clock(RCL_ROS_TIME);
            vision_detections.header.stamp = ros_clock.now();
            vision_detections.frames.emplace_back(detection);
            std::cout << "aaa" << std::endl;
            pub_detection->publish(vision_detections);
        }
        if (packet.has_geometry()) {
            auto geo_msg = constructGeometryMsg(packet.geometry().field());
            pub_geometry->publish(geo_msg);
        }
    }
    grsim_msgs::msg::VisionGeometry constructGeometryMsg(const SSL_GeometryFieldSize &packet_field) {
        auto geometry = grsim_msgs::msg::VisionGeometry();

        geometry.field_length = packet_field.field_length() * TO_METER;
        geometry.field_width = packet_field.field_width() * TO_METER;
        geometry.goal_width = packet_field.goal_width() * TO_METER;
        geometry.goal_depth = packet_field.goal_depth() * TO_METER;
        geometry.boundary_width = static_cast<int>(packet_field.boundary_width() * TO_METER);

        auto lines = packet_field.field_lines();
        for (auto line = lines.begin(); line != lines.end(); line++) {
            auto line_segment = grsim_msgs::msg::FieldLineSegment();

            line_segment.name = line->name();
            line_segment.p1_x = line->p1().x() * TO_METER;
            line_segment.p1_y = line->p1().y() * TO_METER;
            line_segment.p2_x = line->p2().x() * TO_METER;
            line_segment.p2_y = line->p2().y() * TO_METER;
            line_segment.thickness = line->thickness() * TO_METER;
            geometry.field_lines.push_back(line_segment);
        }

        auto arcs = packet_field.field_arcs();
        for (auto arc = arcs.begin(); arc != arcs.end(); arc++) {
            auto circular_arc = grsim_msgs::msg::FieldCircularArc();

            circular_arc.name = arc->name();
            circular_arc.center_x = arc->center().x() * TO_METER;
            circular_arc.center_y = arc->center().y() * TO_METER;
            circular_arc.radius = arc->radius() * TO_METER;
            circular_arc.a1 = arc->a1();
            circular_arc.a2 = arc->a2();
            circular_arc.thickness = arc->thickness() * TO_METER;
            geometry.field_arcs.push_back(circular_arc);
        }

        return geometry;
    }
    grsim_msgs::msg::DetectionFrame constructDetectionFrame(const SSL_DetectionFrame &packet_detection) {
        auto detection_frame = grsim_msgs::msg::DetectionFrame();

        detection_frame.t_capture = packet_detection.t_capture();
        detection_frame.t_sent = packet_detection.t_sent();
        detection_frame.camera_id = packet_detection.camera_id();

        // ball
        auto balls = packet_detection.balls();
        for (auto ball = balls.begin(); ball != balls.end(); ball++) {
            detection_frame.balls.push_back(this->convertToBallTopic(*ball));
        }

        // blue robot
        auto robots_blue = packet_detection.robots_blue();
        for (auto robot = robots_blue.begin(); robot != robots_blue.end(); robot++) {
            detection_frame.robots_blue.push_back(this->convertToRobotTopic(*robot));
        }

        // yellow robot
        auto robots_yellow = packet_detection.robots_yellow();
        for (auto robot = robots_yellow.begin(); robot != robots_yellow.end(); robot++) {
            detection_frame.robots_yellow.push_back(this->convertToRobotTopic(*robot));
        }

        return detection_frame;
    }

    rclcpp::Publisher<grsim_msgs::msg::VisionGeometry>::SharedPtr pub_geometry = nullptr;
    rclcpp::Publisher<grsim_msgs::msg::VisionDetections>::SharedPtr pub_detection = nullptr;

private:
    grsim_msgs::msg::DetectionBall convertToBallTopic(const SSL_DetectionBall &raw_ball) {
        auto detection_ball = grsim_msgs::msg::DetectionBall();

        detection_ball.pose.x = raw_ball.x() * TO_METER;
        detection_ball.pose.y = raw_ball.y() * TO_METER;

        return detection_ball;
    }

    grsim_msgs::msg::DetectionRobot convertToRobotTopic(const SSL_DetectionRobot &raw_robot) {
        auto detection_robot = grsim_msgs::msg::DetectionRobot();

        detection_robot.robot_id = raw_robot.robot_id();
        detection_robot.pose.x = raw_robot.x() * TO_METER;
        detection_robot.pose.y = raw_robot.y() * TO_METER;
        detection_robot.pose.theta = raw_robot.orientation();

        return detection_robot;
    }
};

class RoboCupSSLServerComponent : public rclcpp::Node {
public:
    ROBOCUP_SSL_SERVER_COMPONENT_PUBLIC
    explicit RoboCupSSLServerComponent(const rclcpp::NodeOptions & options) : Node("grsim_server_node", options) {
        converter.pub_geometry =
                this->create_publisher<grsim_msgs::msg::VisionGeometry>("~/raw_vision_geometry", 1);
        converter.pub_detection =
                this->create_publisher<grsim_msgs::msg::VisionDetections>("~/raw_vision_detections", 1);
        has_initilized = true;
    }

    ~RoboCupSSLServerComponent() {}

    bool send(const SSL_WrapperPacket &packet) {
        converter.publishSSLPacket(packet);
        return true;
    }
    bool ok(){
        return has_initilized;
    }

protected:
    bool has_initilized = false;
    FormatConverter converter;
};

#endif
