/*================================================================================

Module Name: SysMsg.h

General Description: ROS messager.

==================================================================================*/

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <set>
#include <string>
#include <thread>

namespace Msg
{

class MessagerObserver
{
public:
  virtual void OnMessage(const std::string &topic, const std::string &payload) = 0;
};

// Messager is thread safe
class Messager
{
public:
  static Messager& GetMessager();

  // Only the first call to init ROS with the node name
  bool Init(int argc, char **argv, const std::string &node_name);

  // Stop ROS. Need call before app exits
  bool Stop();

  // Must create all topics for publisher before calling Publish().
  // Normally CreatePublisherTopics() after Init()
  bool CreatePublisherTopics(const std::set<std::string> &topics);

  // Return false if the topic is not created successfully in CreateTopics()
  bool Publish(const std::string &topic, const std::string &payload);

  // Can subscribe on the same topic more than once, but still one copy
  // of the change on the topic
  bool Subscribe(const std::string &topic, MessagerObserver &observer);

  // Notify observers async in threads until Stop. Not need for publisher only
  bool StartAsync(uint32_t thread_count = 0);

private:
  Messager() = default;

  // No copyable
  Messager(const Messager&) = delete;
  Messager& operator=(const Messager&) = delete;

  static void SignalHandler(int signum);

  bool IsReady() const;
  void Spin();

  class Listener
  {
  public:
    Listener(const std::string &topic,
             rclcpp::Node &node_handle,
             std::recursive_mutex& mutex);
    void Callback(const std_msgs::msg::String::SharedPtr msg);
    std::string get_topic_name() const;
    void AddObserver(MessagerObserver &observer);

  private:
    const std::string topic_;
    std::recursive_mutex& mutex_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    std::set<MessagerObserver*> observers_;
  };

  class MsgNode : public rclcpp::Node
  {
  public:
    MsgNode(const std::string& node_name);
    ~MsgNode();
    void CreatePublisherTopics(const std::set<std::string> &topics);
    bool Publish(const std::string &topic, const std::string &payload);
    bool Subscribe(const std::string &topic, MessagerObserver &observer);

  private:
    std::recursive_mutex mutex_;
    std::set<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publishers_;
    std::set<std::shared_ptr<Listener>> listeners_;
  };

  std::shared_ptr<MsgNode> node_handle_;

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::shared_ptr<std::thread> spin_thread_;
};

} // namespace Msg
