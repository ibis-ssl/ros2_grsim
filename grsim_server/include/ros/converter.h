static const double TO_METER = 0.001;

class FormatConverter
{
public:
  FormatConverter() {vision_detections = ros2_grsim::msg::VisionDetections();}
  void publishSSLPacket(SSL_WrapperPacket packet){
    if (packet.has_detection()) {
      this->append_frame(packet.detection());
      rclcpp::Clock ros_clock(RCL_ROS_TIME);
      vision_detection.header.stamp = ros_clock.now();
      this->pub_detection->publish(vision_detection);
      frame_clear();
    }
    if (packet.has_geometry()) {
      this->publish_geometry(packet.geometry().field());
    }
  }
  void publish_geometry(const SSL_GeometryFieldSize & packet_field)
  {
    auto geometry = ros2_grsim::msg::VisionGeometry();

    geometry.field_length = packet_field.field_length() * TO_METER;
    geometry.field_width = packet_field.field_width() * TO_METER;
    geometry.goal_width = packet_field.goal_width() * TO_METER;
    geometry.goal_depth = packet_field.goal_depth() * TO_METER;
    geometry.boundary_width = static_cast<int>(packet_field.boundary_width() * TO_METER);

    auto lines = packet_field.field_lines();
    for (auto line = lines.begin(); line != lines.end(); line++) {
      auto line_segment = ros2_grsim::msg::FieldLineSegment();

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
      auto circular_arc = ros2_grsim::msg::FieldCircularArc();

      circular_arc.name = arc->name();
      circular_arc.center_x = arc->center().x() * TO_METER;
      circular_arc.center_y = arc->center().y() * TO_METER;
      circular_arc.radius = arc->radius() * TO_METER;
      circular_arc.a1 = arc->a1();
      circular_arc.a2 = arc->a2();
      circular_arc.thickness = arc->thickness() * TO_METER;
      geometry.field_arcs.push_back(circular_arc);
    }

    this->pub_geometry->publish(geometry);
  }
  void append_frame(const SSL_DetectionFrame & packet_detection)
  {
    auto detection_frame = ros2_grsim::msg::DetectionFrame();

    detection_frame.t_capture = packet_detection.t_capture();
    detection_frame.t_sent = packet_detection.t_sent();
    detection_frame.camera_id = packet_detection.camera_id();

    // ball
    auto balls = packet_detection.balls();
    for (auto ball = balls.begin(); ball != balls.end(); ball++) {
      detection_frame.balls.push_back(this->convert_to_ball_topic(*ball));
    }

    // blue robot
    auto robots_blue = packet_detection.robots_blue();
    for (auto robot = robots_blue.begin(); robot != robots_blue.end(); robot++) {
      detection_frame.robots_blue.push_back(this->convert_to_robot_topic(*robot));
    }

    // yellow robot
    auto robots_yellow = packet_detection.robots_yellow();
    for (auto robot = robots_yellow.begin(); robot != robots_yellow.end(); robot++) {
      detection_frame.robots_yellow.push_back(this->convert_to_robot_topic(*robot));
    }

    this->vision_detections.frames.push_back(detection_frame);
  }

  void frame_clear() {this->vision_detections.frames.clear();}

  ros2_grsim::msg::VisionDetections get_frame() {return this->vision_detections;}

private:
  ros2_grsim::msg::DetectionBall convert_to_ball_topic(const SSL_DetectionBall & raw_ball)
  {
    auto detection_ball = ros2_grsim::msg::DetectionBall();

    detection_ball.pose.x = raw_ball.x() * TO_METER;
    detection_ball.pose.y = raw_ball.y() * TO_METER;

    return detection_ball;
  }

  ros2_grsim::msg::DetectionRobot convert_to_robot_topic(const SSL_DetectionRobot & raw_robot)
  {
    auto detection_robot = ros2_grsim::msg::DetectionRobot();

    detection_robot.robot_id = raw_robot.robot_id();
    detection_robot.pose.x = raw_robot.x() * TO_METER;
    detection_robot.pose.y = raw_robot.y() * TO_METER;
    detection_robot.pose.theta = raw_robot.orientation();

    return detection_robot;
  }

  ros2_grsim::msg::VisionDetections vision_detections;
};