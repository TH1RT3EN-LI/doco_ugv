#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <QApplication>
#include <QCloseEvent>
#include <QFocusEvent>
#include <QFontDatabase>
#include <QKeyEvent>
#include <QLabel>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ugv_teleop/keyboard_teleop_common.hpp"

namespace ugv_teleop
{

namespace
{
std::string join_keys(const std::vector<std::string> & keys)
{
  if (keys.empty()) {
    return "-";
  }
  std::string joined;
  for (const auto & key : keys) {
    joined += key;
  }
  return joined;
}

std::string normalize_gui_key(const QKeyEvent * event)
{
  if (event == nullptr) {
    return "";
  }

  const QString text = event->text();
  if (!text.isEmpty()) {
    const QChar c = text.at(0).toLower();
    if (c.isLetter()) {
      return std::string(1, static_cast<char>(c.toLatin1()));
    }
  }

  switch (event->key()) {
    case Qt::Key_Q: return "q";
    case Qt::Key_W: return "w";
    case Qt::Key_E: return "e";
    case Qt::Key_A: return "a";
    case Qt::Key_S: return "s";
    case Qt::Key_D: return "d";
    case Qt::Key_X: return "x";
    case Qt::Key_I: return "i";
    case Qt::Key_O: return "o";
    case Qt::Key_K: return "k";
    case Qt::Key_L: return "l";
    default: return "";
  }
}
}

class KeyboardGuiTeleopNode : public rclcpp::Node
{
public:
  KeyboardGuiTeleopNode()
  : rclcpp::Node("keyboard_gui_teleop_node")
  {
    common_params_ = declare_common_parameters(*this);
    core_ = create_core_from_params(common_params_);
    publisher_ = create_twist_publisher_adapter(*this, common_params_);

    rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
    read_only_descriptor.read_only = true;
    window_title_ = this->declare_parameter<std::string>(
      "window_title", "UGV Keyboard Teleop", read_only_descriptor);
    focus_loss_zero_cmd_ = this->declare_parameter<bool>(
      "focus_loss_zero_cmd", true, read_only_descriptor);

    const auto publish_period = std::chrono::duration<double>(
      1.0 / std::max(1.0, common_params_.publish_rate));
    publish_timer_ = this->create_wall_timer(
      publish_period,
      std::bind(&KeyboardGuiTeleopNode::publish_timer_callback, this));

    status_text_ = k_common_keyboard_message;
  }

  ~KeyboardGuiTeleopNode() override
  {
    shutdown();
  }

  std::string window_title() const
  {
    return window_title_;
  }

  void handle_key_press(const std::string & key)
  {
    if (key.empty()) {
      return;
    }
    std::lock_guard<std::mutex> lock(pressed_keys_mutex_);
    if (pressed_keys_.count(key) > 0U) {
      return;
    }
    if (core_->handle_key_press(key, std::chrono::steady_clock::now())) {
      pressed_keys_.insert(key);
    }
  }

  void handle_key_release(const std::string & key)
  {
    if (key.empty()) {
      return;
    }
    std::lock_guard<std::mutex> lock(pressed_keys_mutex_);
    pressed_keys_.erase(key);
    core_->handle_key_release(key, std::chrono::steady_clock::now());
  }

  void handle_focus_lost()
  {
    {
      std::lock_guard<std::mutex> lock(pressed_keys_mutex_);
      pressed_keys_.clear();
    }
    core_->clear_move_keys(std::chrono::steady_clock::now());
    if (focus_loss_zero_cmd_) {
      publisher_->publish_zero(core_.get());
    }
  }

  std::string status_text() const
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_text_;
  }

  void shutdown()
  {
    if (shutdown_done_.exchange(true)) {
      return;
    }
    core_->emergency_stop(std::chrono::steady_clock::now());
    if (rclcpp::ok()) {
      publisher_->publish_zero(core_.get());
    }
  }

private:
  void publish_timer_callback()
  {
    const auto command = core_->snapshot(std::chrono::steady_clock::now());
    publisher_->publish(command);
    update_status_text(command);
  }

  void update_status_text(const TeleopCommand & command)
  {
    const std::string text =
      std::string(k_common_keyboard_message) +
      "\nCurrent speed: linear " + std::to_string(command.linear_speed) +
      " m/s | angular " + std::to_string(command.angular_speed) + " rad/s\n" +
      "Current keys: " + join_keys(command.active_keys) + "\n" +
      "Focus loss will stop motion";
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_text_ = text;
  }

  CommonTeleopParams common_params_;
  std::shared_ptr<KeyboardTeleopCore> core_;
  std::shared_ptr<TwistPublisherAdapter> publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::string window_title_;
  bool focus_loss_zero_cmd_{true};

  mutable std::mutex status_mutex_;
  std::string status_text_;

  std::mutex pressed_keys_mutex_;
  std::set<std::string> pressed_keys_;
  std::atomic<bool> shutdown_done_{false};
};

class KeyboardTeleopWindow : public QWidget
{
public:
  explicit KeyboardTeleopWindow(std::shared_ptr<KeyboardGuiTeleopNode> teleop_node)
  : teleop_node_(std::move(teleop_node))
  {
    this->setWindowTitle(QString::fromStdString(teleop_node_->window_title()));
    this->setFixedSize(460, 280);
    this->setFocusPolicy(Qt::StrongFocus);

    auto * layout = new QVBoxLayout(this);
    layout->setContentsMargins(16, 16, 16, 16);

    status_label_ = new QLabel(this);
    status_label_->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    status_label_->setWordWrap(true);
    QFont font = QFontDatabase::systemFont(QFontDatabase::FixedFont);
    font.setPointSize(11);
    status_label_->setFont(font);
    layout->addWidget(status_label_);

    status_timer_ = new QTimer(this);
    connect(status_timer_, &QTimer::timeout, this, &KeyboardTeleopWindow::refresh_status);
    status_timer_->start(100);

    ros_alive_timer_ = new QTimer(this);
    connect(ros_alive_timer_, &QTimer::timeout, this, &KeyboardTeleopWindow::check_ros_alive);
    ros_alive_timer_->start(100);

    refresh_status();
  }

protected:
  void keyPressEvent(QKeyEvent * event) override
  {
    if (event->isAutoRepeat()) {
      event->accept();
      return;
    }
    const std::string key = normalize_gui_key(event);
    if (!key.empty()) {
      teleop_node_->handle_key_press(key);
    }
    QWidget::keyPressEvent(event);
  }

  void keyReleaseEvent(QKeyEvent * event) override
  {
    if (event->isAutoRepeat()) {
      event->accept();
      return;
    }
    const std::string key = normalize_gui_key(event);
    if (!key.empty()) {
      teleop_node_->handle_key_release(key);
    }
    QWidget::keyReleaseEvent(event);
  }

  void focusOutEvent(QFocusEvent * event) override
  {
    teleop_node_->handle_focus_lost();
    QWidget::focusOutEvent(event);
  }

  void closeEvent(QCloseEvent * event) override
  {
    teleop_node_->shutdown();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    QWidget::closeEvent(event);
  }

private:
  void refresh_status()
  {
    status_label_->setText(QString::fromStdString(teleop_node_->status_text()));
  }

  void check_ros_alive()
  {
    if (!rclcpp::ok()) {
      this->close();
    }
  }

  std::shared_ptr<KeyboardGuiTeleopNode> teleop_node_;
  QLabel * status_label_{nullptr};
  QTimer * status_timer_{nullptr};
  QTimer * ros_alive_timer_{nullptr};
};

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;

  std::shared_ptr<ugv_teleop::KeyboardGuiTeleopNode> teleop_node;
  std::thread spinner;

  try {
    teleop_node = std::make_shared<ugv_teleop::KeyboardGuiTeleopNode>();
    spinner = std::thread(
      [teleop_node]() {
        rclcpp::spin(teleop_node);
      });

    QApplication app(argc, argv);
    ugv_teleop::KeyboardTeleopWindow window(teleop_node);
    window.show();
    window.activateWindow();
    window.raise();
    window.setFocus();

    exit_code = app.exec();
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      rclcpp::get_logger("keyboard_gui_teleop_node"),
      "Unhandled exception: %s",
      exception.what());
    exit_code = 1;
  }

  if (teleop_node) {
    teleop_node->shutdown();
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  if (spinner.joinable()) {
    spinner.join();
  }
  return exit_code;
}
