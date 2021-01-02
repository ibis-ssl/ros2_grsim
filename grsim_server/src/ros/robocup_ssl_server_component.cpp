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
  \file    robocup_ssl_server.cpp
  \brief   C++ Implementation: robocup_ssl_server
  \author  Stefan Zickler, 2009
  \author  Jan Segre, 2012
*/
//========================================================================
#include "robocup_ssl_server_component.h"
#include <QtNetwork>
#include <iostream>
#include "logger.h"


RoboCupSSLServerComponent::RoboCupSSLServerComponent(const rclcpp::NodeOptions & options) : Node("grsim_server_node", options){
      converter.pub_geometry =
        this->create_publisher<grsim_msgs::msg::VisionGeometry>("~/raw_vision_geometry", 1);
      converter.pub_detection =
        this->create_publisher<grsim_msgs::msg::VisionDetections>("~/raw_vision_detections", 1);
      has_initilized = true;
    }