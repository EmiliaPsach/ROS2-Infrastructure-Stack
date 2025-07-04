// #include <gtest/gtest.h>
// #include <QApplication>
// #include <QKeyEvent>
// #include <QMouseEvent>
// #include <QTest>
// #include <thread>        // For std::this_thread::sleep_for
// #include <chrono>

// #include "gui_pose_issuer/gui_pose_issuer.hpp"
// #include "rclcpp/rclcpp.hpp"

// using namespace gui_pose_issuer;

// // Test fixture for GuiPoseIssuer node
// class GuiPoseIssuerTest : public ::testing::Test {
//  protected:
//   // Create QApplication once for all tests
//   static void SetUpTestSuite() {
//     int argc = 0;
//     app_ = new QApplication(argc, nullptr);
//   }

//   static void TearDownTestSuite() {
//     delete app_;
//     app_ = nullptr;
//   }

//   void SetUp() override {
//     node_ = std::make_shared<GuiPoseIssuer>();
//     node_->view_->show();

//     // Process Qt events to fully initialize GUI
//     QTest::qWait(100);
//     QCoreApplication::processEvents();

//     // Small delay to avoid race conditions (optional)
//     std::this_thread::sleep_for(std::chrono::milliseconds(50));
//   }

//   void TearDown() override {
//     node_.reset();
//   }

//   std::shared_ptr<GuiPoseIssuer> node_;
//   static QApplication* app_;
// };

// QApplication* GuiPoseIssuerTest::app_ = nullptr;

// // Test that publish_pose does not throw and executes successfully
// TEST_F(GuiPoseIssuerTest, PublishesPoseOnValidClick) {
//   QPointF point(0.5, -0.5);
//   EXPECT_NO_THROW(node_->publish_pose(point));
// }

// // Test that pressing spacebar enables clock mode
// TEST_F(GuiPoseIssuerTest, ClockModeTogglesOnSpacebar) {
//   EXPECT_TRUE(node_->clock_mode_enabled_);

//   QKeyEvent event(QEvent::KeyPress, Qt::Key_Space, Qt::NoModifier);
//   node_->eventFilter(nullptr, &event);

//   EXPECT_TRUE(node_->clock_mode_enabled_);
// }

// // Test that clicking disables clock mode
// TEST_F(GuiPoseIssuerTest, MouseClickDisablesClockMode) {
//   EXPECT_TRUE(node_->clock_mode_enabled_);

//   QPoint center = node_->view_->rect().center();
//   QMouseEvent event(QEvent::MouseButtonPress, center, Qt::LeftButton,
//                     Qt::LeftButton, Qt::NoModifier);

//   QApplication::sendEvent(node_->view_, &event);

//   EXPECT_FALSE(node_->clock_mode_enabled_);
// }

// int main(int argc, char **argv) {
//   // Initialize ROS2 once for all tests
//   rclcpp::init(argc, argv);

//   ::testing::InitGoogleTest(&argc, argv);
//   int ret = RUN_ALL_TESTS();

//   // Shutdown ROS2 after all tests
//   rclcpp::shutdown();

//   return ret;
// }
