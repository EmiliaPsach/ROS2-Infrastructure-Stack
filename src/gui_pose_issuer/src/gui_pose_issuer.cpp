/* Implements the class interface for gui_pose_issuer */

#include "gui_pose_issuer/gui_pose_issuer.hpp"

#include <QApplication>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QKeyEvent>
#include <QMouseEvent>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace gui_pose_issuer {

// Define default topic name
constexpr const char* GuiPoseIssuer::DEFAULT_TOPIC_NAME;

// Constructor initializes parameters, GUI, and ROS publisher
GuiPoseIssuer::GuiPoseIssuer()
    : rclcpp::Node("gui_pose_issuer"),
      clock_mode_enabled_(true) {
  // Declare and retrieve parameters
  this->declare_parameter("frame_id", "map");
  this->declare_parameter("topic_name", DEFAULT_TOPIC_NAME);

  frame_id_ = this->get_parameter("frame_id").as_string();
  topic_name_ = this->get_parameter("topic_name").as_string();

  // Create publisher for pose messages
  pose_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name_, 10);

  // Set up simple 2D scene and view
  scene_ = new QGraphicsScene();
  view_ = new QGraphicsView(scene_);

  // Define scene rectangle
  view_->setSceneRect(-1.1, -1.1, 2.2, 2.2);
  view_->setRenderHint(QPainter::Antialiasing);

  // Draw clock face circle
  QPen pen(Qt::black);
  pen.setWidthF(0.02);
  QBrush brush(Qt::white);
  scene_->addEllipse(-1.0, -1.0, 2.0, 2.0, pen, brush);

  // Draw hour ticks
  for (int i = 0; i < 12; ++i) {
    double angle = i * M_PI / 6;  // 30 degrees per hour
    double inner_r = 0.9;
    double outer_r = 1.0;
    QPointF inner(inner_r * std::cos(angle), inner_r * std::sin(angle));
    QPointF outer(outer_r * std::cos(angle), outer_r * std::sin(angle));
    scene_->addLine(QLineF(inner, outer), pen);
  }

  // Install event filter to capture mouse clicks and key presses
  view_->installEventFilter(this);

  // Make sure the view fits the scene rect exactly, scaling everything
  view_->fitInView(scene_->sceneRect(), Qt::KeepAspectRatio);

  // Set window size explicitly (optional)
  view_->setFixedSize(400, 400);

  view_->show();
}

// Handles mouse clicks and key presses
bool GuiPoseIssuer::eventFilter(QObject* obj, QEvent* event) {
  if (event->type() == QEvent::MouseButtonPress) {
    // Handle mouse click
    QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
    QPointF pos = view_->mapToScene(mouseEvent->pos());

    if (pos.manhattanLength() <= 1.0) {
      publish_pose(pos);             // Send pose command
      clock_mode_enabled_ = false;  // Switch off clock mode
    }
    return true;
  } else if (event->type() == QEvent::KeyPress) {
    // Handle key press
    QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);
    if (keyEvent->key() == Qt::Key_Space) {
      clock_mode_enabled_ = true;
      RCLCPP_INFO(this->get_logger(), "Re-enabled clock pose following.");
    }
    return true;
  }
  return QObject::eventFilter(obj, event);
}

// Publishes a pose message to the robot
void GuiPoseIssuer::publish_pose(const QPointF& point) {
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = frame_id_;
  msg.pose.position.x = point.x();
  msg.pose.position.y = point.y();
  msg.pose.position.z = 0.0;

  // No orientation is specified (identity quaternion)
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 1.0;

  pose_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Published GUI pose: (%.2f, %.2f)",
              point.x(), point.y());
}

}  // namespace gui_pose_issuer
