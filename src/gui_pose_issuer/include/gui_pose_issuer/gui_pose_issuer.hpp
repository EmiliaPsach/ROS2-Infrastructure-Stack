/* Declares the class interface for the gui_pose_issuer package */

#ifndef GUI_POSE_ISSUER_HPP_
#define GUI_POSE_ISSUER_HPP_

#include <memory>
#include <string>

#include <QGraphicsScene>
#include <QGraphicsView>
#include <QObject>
#include <QPointF>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace gui_pose_issuer {

/**
 * @brief GUI node that allows a human to click on a clock face to command robot poses,
 *        or press spacebar to clear GUI command and revert to clock following.
 */
class GuiPoseIssuer : public rclcpp::Node, public QObject {
 public:
  /**
   * @brief Constructor - initializes ROS node, GUI components, and publisher.
   */
  GuiPoseIssuer();

  /**
   * @brief Destructor
   */
  ~GuiPoseIssuer() = default;

  /// Default ROS topic name to publish target poses
  static constexpr const char* DEFAULT_TOPIC_NAME = "/target_pose_gui";

  /**
   * @brief Event filter to handle mouse clicks and key presses on the GUI view.
   * @param obj The object that received the event.
   * @param event The event to filter.
   * @return True if event is handled, otherwise false.
   */
  bool eventFilter(QObject* obj, QEvent* event) override;

  /**
   * @brief Publish a pose message based on the clicked GUI point.
   * @param point The clicked position in the clock coordinate frame (normalized [-1,1]).
   */
  void publish_pose(const QPointF& point);

  // Public members for easier testing

  /// ROS frame id for published pose
  std::string frame_id_;

  /// ROS topic name to publish poses
  std::string topic_name_;

  /// Flag indicating whether robot is following clock poses (true) or user GUI pose (false)
  bool clock_mode_enabled_;

  /// ROS2 publisher for PoseStamped messages
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  // ROS2 publisher for mode following (space bar enabled)
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mode_publisher_;

  /// Qt graphics components for GUI display
  QGraphicsScene* scene_;
  QGraphicsView* view_;
};

}  // namespace gui_pose_issuer

#endif  // GUI_POSE_ISSUER_HPP_
