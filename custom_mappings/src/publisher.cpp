#include <cmath>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <custom_mappings/msg/motor_status.hpp>
#include <custom_mappings/msg/detections.hpp>
#include <custom_mappings/msg/detections2_d.hpp>
#include <custom_mappings/msg/bounding_boxes2_d.hpp>
#include <custom_mappings/msg/bounding_boxes3_d.hpp>
#include <custom_mappings/msg/trajectory.hpp>
#include <custom_mappings/msg/force_vector.hpp>
#include <custom_mappings/msg/gps_reading.hpp>
#include <custom_mappings/msg/rigid_bodies.hpp>

using namespace std::chrono_literals;

class CustomPublisher : public rclcpp::Node {
public:
  CustomPublisher() : Node("custom_publisher"), tick_(0) {
    motor_pub_ = create_publisher<custom_mappings::msg::MotorStatus>("/motor/status", 10);
    det_pub_ = create_publisher<custom_mappings::msg::Detections>("/detections", 10);
    det2d_pub_ = create_publisher<custom_mappings::msg::Detections2D>("/detections_2d", 10);
    bbox2d_pub_ = create_publisher<custom_mappings::msg::BoundingBoxes2D>("/bounding_boxes_2d", 10);
    bbox3d_pub_ = create_publisher<custom_mappings::msg::BoundingBoxes3D>("/bounding_boxes_3d", 10);
    traj_pub_ = create_publisher<custom_mappings::msg::Trajectory>("/trajectory", 10);
    rb_pub_ = create_publisher<custom_mappings::msg::RigidBodies>("/rigidbodies", 10);
    gps_pub_ = create_publisher<custom_mappings::msg::GpsReading>("/gps", 10);
    force_pub_ = create_publisher<custom_mappings::msg::ForceVector>("/force", 10);
    timer_ = create_wall_timer(100ms, [this] { publish(); });
  }

private:
  void publish() {
    double t = tick_ * 0.1;
    tick_++;
    auto stamp = now();

    // MotorStatus: leaf scalars + text log
    auto motor = custom_mappings::msg::MotorStatus();
    motor.header.stamp = stamp;
    motor.header.frame_id = "base_link";
    motor.temperature = 40.0 + 5.0 * std::sin(t);
    motor.rpm = 3000.0 + 500.0 * std::cos(t * 0.5);
    motor.status_text = motor.rpm > 2800.0 ? "running" : "idle";
    motor_pub_->publish(motor);

    // Detections: collection each with Points3D + labels
    auto det_msg = custom_mappings::msg::Detections();
    det_msg.header.stamp = stamp;
    det_msg.header.frame_id = "camera_link";

    custom_mappings::msg::Detection d1;
    d1.position.x = 1.0 + 0.5 * std::sin(t);
    d1.position.y = 2.0;
    d1.position.z = 0.0;
    d1.label = "person";
    d1.confidence = 0.95;

    custom_mappings::msg::Detection d2;
    d2.position.x = 4.0;
    d2.position.y = 5.0 + std::cos(t);
    d2.position.z = 1.0;
    d2.label = "car";
    d2.confidence = 0.87;

    det_msg.detections = {d1, d2};
    det_msg.count = static_cast<int32_t>(det_msg.detections.size());
    det_pub_->publish(det_msg);

    // RigidBodies: expansion each + entity_key + composite inside
    auto rb_msg = custom_mappings::msg::RigidBodies();
    rb_msg.header.stamp = stamp;
    rb_msg.header.frame_id = "mocap";
    rb_msg.frame_number = tick_;

    custom_mappings::msg::RigidBody body1;
    body1.rigid_body_name = "drone";
    body1.pose.position.x = 2.0 * std::cos(t * 0.3);
    body1.pose.position.y = 2.0 * std::sin(t * 0.3);
    body1.pose.position.z = 1.5;
    body1.pose.orientation.w = std::cos(t * 0.1);
    body1.pose.orientation.z = std::sin(t * 0.1);

    geometry_msgs::msg::Point m1, m2, m3;
    m1.x = body1.pose.position.x + 0.1;
    m1.y = body1.pose.position.y;
    m1.z = body1.pose.position.z;
    m2.x = body1.pose.position.x - 0.1;
    m2.y = body1.pose.position.y + 0.1;
    m2.z = body1.pose.position.z;
    m3.x = body1.pose.position.x;
    m3.y = body1.pose.position.y - 0.1;
    m3.z = body1.pose.position.z;
    body1.markers = {m1, m2, m3};

    custom_mappings::msg::RigidBody body2;
    body2.rigid_body_name = "target";
    body2.pose.position.x = 0.0;
    body2.pose.position.y = 3.0;
    body2.pose.position.z = 0.5 + 0.2 * std::sin(t);
    body2.pose.orientation.w = 1.0;

    geometry_msgs::msg::Point m4;
    m4.x = 0.0;
    m4.y = 3.0;
    m4.z = body2.pose.position.z;
    body2.markers = {m4};

    rb_msg.rigidbodies = {body1, body2};
    rb_pub_->publish(rb_msg);

    // GpsReading: GeoPoints with nested fix struct
    auto gps = custom_mappings::msg::GpsReading();
    gps.header.stamp = stamp;
    gps.header.frame_id = "gps_link";
    gps.fix.latitude = 37.7749 + 0.0001 * std::sin(t * 0.2);
    gps.fix.longitude = -122.4194 + 0.0001 * std::cos(t * 0.2);
    gps.fix.altitude = 10.0 + 2.0 * std::sin(t * 0.5);
    gps_pub_->publish(gps);

    // ForceVector: single Arrows3D + magnitude scalar
    auto force = custom_mappings::msg::ForceVector();
    force.header.stamp = stamp;
    force.header.frame_id = "base_link";
    force.arrow.origin.x = 0.0;
    force.arrow.origin.y = 0.0;
    force.arrow.origin.z = 1.0;
    force.arrow.direction.x = std::cos(t) * 5.0;
    force.arrow.direction.y = std::sin(t) * 5.0;
    force.arrow.direction.z = 2.0;
    force.magnitude = std::sqrt(
      force.arrow.direction.x * force.arrow.direction.x +
      force.arrow.direction.y * force.arrow.direction.y +
      force.arrow.direction.z * force.arrow.direction.z);
    force_pub_->publish(force);

    // Detections2D: collection each → Points2D with labels
    auto det2d = custom_mappings::msg::Detections2D();
    det2d.header.stamp = stamp;
    det2d.header.frame_id = "camera_link";

    custom_mappings::msg::Detection2D p1;
    p1.position.x = 320.0 + 50.0 * std::sin(t);
    p1.position.y = 240.0 + 30.0 * std::cos(t);
    p1.label = "pedestrian";
    p1.confidence = 0.92;

    custom_mappings::msg::Detection2D p2;
    p2.position.x = 100.0;
    p2.position.y = 400.0 + 20.0 * std::sin(t * 0.7);
    p2.label = "cyclist";
    p2.confidence = 0.78;

    custom_mappings::msg::Detection2D p3;
    p3.position.x = 500.0 + 10.0 * std::cos(t * 1.3);
    p3.position.y = 150.0;
    p3.label = "vehicle";
    p3.confidence = 0.99;

    det2d.detections = {p1, p2, p3};
    det2d.count = static_cast<int32_t>(det2d.detections.size());
    det2d_pub_->publish(det2d);

    // BoundingBoxes2D: collection each → Boxes2D with labels
    auto bbox2d = custom_mappings::msg::BoundingBoxes2D();
    bbox2d.header.stamp = stamp;
    bbox2d.header.frame_id = "camera_link";

    custom_mappings::msg::BoundingBox2D b2d1;
    b2d1.center_x = 320.0 + 30.0 * std::sin(t);
    b2d1.center_y = 240.0;
    b2d1.half_width = 40.0;
    b2d1.half_height = 80.0;
    b2d1.label = "person";

    custom_mappings::msg::BoundingBox2D b2d2;
    b2d2.center_x = 500.0;
    b2d2.center_y = 300.0 + 20.0 * std::cos(t);
    b2d2.half_width = 60.0;
    b2d2.half_height = 40.0;
    b2d2.label = "car";

    bbox2d.boxes = {b2d1, b2d2};
    bbox2d_pub_->publish(bbox2d);

    // BoundingBoxes3D: collection each → Boxes3D with labels
    auto bbox3d = custom_mappings::msg::BoundingBoxes3D();
    bbox3d.header.stamp = stamp;
    bbox3d.header.frame_id = "base_link";

    custom_mappings::msg::BoundingBox3D b3d1;
    b3d1.center_x = 5.0 + std::sin(t * 0.5);
    b3d1.center_y = 0.0;
    b3d1.center_z = 0.9;
    b3d1.half_width = 0.9;
    b3d1.half_height = 0.9;
    b3d1.half_depth = 2.2;
    b3d1.label = "sedan";

    custom_mappings::msg::BoundingBox3D b3d2;
    b3d2.center_x = 8.0;
    b3d2.center_y = 3.0 + std::cos(t * 0.3);
    b3d2.center_z = 1.5;
    b3d2.half_width = 1.2;
    b3d2.half_height = 1.5;
    b3d2.half_depth = 3.0;
    b3d2.label = "truck";

    bbox3d.boxes = {b3d1, b3d2};
    bbox3d_pub_->publish(bbox3d);

    // Trajectory: LineStrips3D from waypoints
    auto traj = custom_mappings::msg::Trajectory();
    traj.header.stamp = stamp;
    traj.header.frame_id = "map";

    int num_points = 20;
    for (int i = 0; i < num_points; i++) {
      geometry_msgs::msg::Point wp;
      double s = t + i * 0.3;
      wp.x = 2.0 * std::cos(s * 0.4);
      wp.y = 2.0 * std::sin(s * 0.4);
      wp.z = 0.1 * i;
      traj.waypoints.push_back(wp);
    }
    traj_pub_->publish(traj);

  }

  rclcpp::Publisher<custom_mappings::msg::MotorStatus>::SharedPtr motor_pub_;
  rclcpp::Publisher<custom_mappings::msg::Detections>::SharedPtr det_pub_;
  rclcpp::Publisher<custom_mappings::msg::Detections2D>::SharedPtr det2d_pub_;
  rclcpp::Publisher<custom_mappings::msg::BoundingBoxes2D>::SharedPtr bbox2d_pub_;
  rclcpp::Publisher<custom_mappings::msg::BoundingBoxes3D>::SharedPtr bbox3d_pub_;
  rclcpp::Publisher<custom_mappings::msg::Trajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<custom_mappings::msg::RigidBodies>::SharedPtr rb_pub_;
  rclcpp::Publisher<custom_mappings::msg::GpsReading>::SharedPtr gps_pub_;
  rclcpp::Publisher<custom_mappings::msg::ForceVector>::SharedPtr force_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int tick_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomPublisher>());
  rclcpp::shutdown();
  return 0;
}
