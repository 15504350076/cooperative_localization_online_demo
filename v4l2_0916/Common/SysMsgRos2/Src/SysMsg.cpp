#include "SysMsg/SysMsg.h"

#include <signal.h>

namespace Msg
{

const uint32_t MESSAGE_QUEUE_SIZE = 1000;
const int32_t WAITING_TOPIC_READY_TIME_S = 3;

Messager& Messager::GetMessager()
{
  static Messager instance;
  return instance;
}

bool Messager::Init(int argc, char **argv, const std::string &node_name)
{
  if (!node_handle_) {
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    node_handle_ = std::make_shared<MsgNode>(node_name);
    signal(SIGINT, Messager::SignalHandler);
  }
  return rclcpp::ok();
}

bool Messager::Stop()
{
  if (!IsReady()) {
    return false;
  }

  rclcpp::shutdown();
  if (executor_) {
    spin_thread_->join();
    spin_thread_.reset();
    executor_.reset();
  }
  node_handle_.reset();
  return true;
}

bool Messager::CreatePublisherTopics(const std::set<std::string> &topics)
{
  if (!IsReady()) {
    return false;
  }

  node_handle_->CreatePublisherTopics(topics);
  // Need wait a while for publisher ready. Otherwise the message may be lost
  // if publishing quickly after creating. From ROS1, still need in ROS2?
  rclcpp::Duration duration(WAITING_TOPIC_READY_TIME_S, 0);
  return true;
}

bool Messager::Publish(const std::string &topic, const std::string &payload)
{
  if (!IsReady()) {
    return false;
  }

  return node_handle_->Publish(topic, payload);
}

bool Messager::Subscribe(const std::string &topic, MessagerObserver &observer)
{
  if (!IsReady()) {
    return false;
  }

  return node_handle_->Subscribe(topic, observer);
}

bool Messager::StartAsync(uint32_t thread_count)
{
  if (!IsReady()) {
    return false;
  }

  // Start already
  if (executor_) {
    return true;
  }

  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions(), thread_count);
  executor_->add_node(node_handle_);
  spin_thread_ = std::make_shared<std::thread>(&Messager::Spin, this);
  return true;
}

void Messager::SignalHandler(int signum)
{
  GetMessager().Stop();
  exit(0);
}

bool Messager::IsReady() const
{
  return rclcpp::ok() && node_handle_;
}

void Messager::Spin()
{
  executor_->spin();
}

Messager::Listener::Listener(const std::string &topic,
                             rclcpp::Node &node_handle,
                             std::recursive_mutex& mutex)
  : topic_(topic),
    mutex_(mutex)
{
  auto callback_group = node_handle.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto options = rclcpp::SubscriptionOptions();
  options.callback_group = callback_group;
  subscriber_ = node_handle.create_subscription<std_msgs::msg::String>(
    topic, MESSAGE_QUEUE_SIZE,
    [this](const std_msgs::msg::String::SharedPtr msg) { this->Callback(msg); },
    options);
}

std::string Messager::Listener::get_topic_name() const
{
  return subscriber_->get_topic_name();
}

void Messager::Listener::Callback(const std_msgs::msg::String::SharedPtr msg)
{
  // Copy to local for thread safe. The observers may be added in another thread
  std::set<MessagerObserver*> observers_copy;
  {
    // Not put the mutex protection to the begining of this function,
    // otherwise all OnMessage() will be called one by one
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    observers_copy = observers_;
  }

  for (auto &each : observers_copy) {
    each->OnMessage(topic_, msg->data);
  }
}

void Messager::Listener::AddObserver(MessagerObserver &observer)
{
  std::lock_guard<std::recursive_mutex> guard(mutex_);
  observers_.emplace(&observer);
}

Messager::MsgNode::MsgNode(const std::string& node_name)
  : Node(node_name)
{
}

Messager::MsgNode::~MsgNode()
{
  publishers_.clear();
  listeners_.clear();
}

void  Messager::MsgNode::CreatePublisherTopics(const std::set<std::string> &topics)
{
  std::lock_guard<std::recursive_mutex> guard(mutex_);

  for (auto& topic : topics) {
    bool existed = false;
    for (auto &publisher : publishers_) {
      if (publisher->get_topic_name() == "/" + topic) {
        existed = true;
        break;
      }
    }

    if (!existed) {
      auto publisher = this->create_publisher<std_msgs::msg::String>(
        topic, MESSAGE_QUEUE_SIZE);
      publishers_.emplace(publisher);
    }
  }
}

bool Messager::MsgNode::Publish(const std::string &topic, const std::string &payload)
{
  std::lock_guard<std::recursive_mutex> guard(mutex_);

  for (auto &each : publishers_) {
    if (each->get_topic_name() == "/" + topic) {
      auto msg = std_msgs::msg::String();
      msg.data = payload;
      each->publish(msg);
      return true;
    }
  }

  return false;
}

bool Messager::MsgNode::Subscribe(const std::string &topic, MessagerObserver &observer)
{
  std::lock_guard<std::recursive_mutex> guard(mutex_);

  for (auto &each : listeners_) {
    if (each->get_topic_name() == "/" + topic) {
      each->AddObserver(observer);
      return true;
    }
  }

  auto listener = std::make_shared<Listener>(topic, *this, mutex_);
  listener->AddObserver(observer);
  listeners_.emplace(listener);
  return false;
}

} // namespace Msg
