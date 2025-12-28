#include <QVBoxLayout>
#include <QDir>
#include <rviz_common/display_context.hpp>
#include <benchmarking_3dcpp_rviz_plugin/benchmarking_3dcpp_panel.hpp>

namespace benchmarking_3dcpp_panel
{
Benchmarking3DCPPPanel::Benchmarking3DCPPPanel(QWidget * parent) : Panel(parent)
{
  // Create a label and a button, displayed vertically (the V in VBox means vertical)
  const auto layout = new QVBoxLayout(this);

  // Create and add the combo box
  folder_combo_box_ = new QComboBox();
  layout->addWidget(folder_combo_box_);

  label_ = new QLabel("Select an H5 file to load.");
  button_ = new QPushButton("Load and Visualize!");
  layout->addWidget(label_);
  layout->addWidget(button_);

  // Connect the event of when the button is released to our callback,
  // so pressing the button results in the callback being called.
  QObject::connect(button_, &QPushButton::released, this, &Benchmarking3DCPPPanel::publishSelectedH5File);

  // Populate the combo box with folder names
  populateH5FileComboBox();
}

Benchmarking3DCPPPanel::~Benchmarking3DCPPPanel() = default;

void Benchmarking3DCPPPanel::onInitialize()
{
  // Access the abstract ROS Node and
  // in the process lock it for exclusive use until the method is done.
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

  // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
  // (as per normal rclcpp code)
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();

  h5_file_publisher_ = node->create_publisher<std_msgs::msg::String>("/load_and_visualize_h5", rclcpp::QoS(rclcpp::KeepLast(1)));
}

// When the subscriber gets a message, this callback is triggered,
// and then we copy its data into the widget's label
void Benchmarking3DCPPPanel::topicCallback(const std_msgs::msg::String& msg)
{
  label_->setText(QString(msg.data.c_str()));
}

// When the widget's button is pressed, this callback is triggered,
// and then we publish a new message on our topic.
void Benchmarking3DCPPPanel::buttonActivated()
{
  // Get the currently selected folder name from the combo box
  QString selected_folder = folder_combo_box_->currentText();

  auto message = std_msgs::msg::String();
  message.data = "Button clicked! Selected folder: " + selected_folder.toStdString();
  h5_file_publisher_->publish(message);
  
  // You can also update the label to show the selection
  label_->setText("Selected: " + selected_folder);
}

void Benchmarking3DCPPPanel::populateH5FileComboBox()
{
  try
  {
    // get the shared folder
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("benchmarking_3dcpp");

    // enter the output subfolder
    QDir output_dir(QString::fromStdString(package_share_directory + "/output"));

    if (!output_dir.exists())
    {
      RCLCPP_WARN(rclcpp::get_logger("Benchmarking3DCPPPanel"), "Output directory does not exist: %s", output_dir.path().toStdString().c_str());
      folder_combo_box_->addItem("No 'output' directory found");
      return;
    }

    // we only want .h5 files
    output_dir.setNameFilters(QStringList() << "*.h5");
    output_dir.setFilter(QDir::Files | QDir::NoDotAndDotDot);

    QStringList h5_files = output_dir.entryList();

    folder_combo_box_->clear();

    if (h5_files.isEmpty())
    {
      folder_combo_box_->addItem("No .h5 files found");
    }
    else
    {
      folder_combo_box_->addItems(h5_files);
    }
  }
  catch (const std::exception& e)
  {
    // 如果包找不到或其他错误，打印错误信息
    RCLCPP_ERROR(rclcpp::get_logger("Benchmarking3DCPPPanel"), "Failed to find package share directory: %s", e.what());
    folder_combo_box_->addItem("Error finding package");
  }

}

// 新增的函数实现
void Benchmarking3DCPPPanel::publishSelectedH5File()
{
  QString selected_file = folder_combo_box_->currentText();

  // 检查是否是有效的文件名（而不是错误提示）
  if (selected_file.contains("No") || selected_file.contains("Error"))
  {
    RCLCPP_WARN(rclcpp::get_logger("Benchmarking3DCPPPanel"), "Cannot publish, no valid file selected.");
    label_->setText("Error: No valid file selected.");
    return;
  }

  auto message = std_msgs::msg::String();
  message.data = selected_file.toStdString();

  h5_file_publisher_->publish(message);

  // print to the rviz screen
  label_->setText("Published: " + selected_file);
  
  // print to the rviz terminal
  // RCLCPP_INFO(rclcpp::get_logger("Benchmarking3DCPPPanel"), "Published H5 file '%s' to topic '/load_and_visualize_h5'", message.data.c_str());
}

}  // namespace benchmarking_3dcpp_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(benchmarking_3dcpp_panel::Benchmarking3DCPPPanel, rviz_common::Panel)
