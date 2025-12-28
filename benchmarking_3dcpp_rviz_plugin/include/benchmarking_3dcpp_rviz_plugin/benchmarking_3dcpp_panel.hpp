#pragma once

#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>
#include <QStringList>
#include <QDir>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace benchmarking_3dcpp_panel
{
class Benchmarking3DCPPPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit Benchmarking3DCPPPanel(QWidget * parent = 0);
  ~Benchmarking3DCPPPanel() override;

  void onInitialize() override;

protected:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr h5_file_publisher_;

  void topicCallback(const std_msgs::msg::String& msg);

  QLabel * label_;
  QPushButton * button_;
  QComboBox * folder_combo_box_;

private Q_SLOTS:
  void buttonActivated();

private: 

  void publishSelectedH5File();
  
  void populateH5FileComboBox();
};

}  // namespace benchmarking_3dcpp_panel
