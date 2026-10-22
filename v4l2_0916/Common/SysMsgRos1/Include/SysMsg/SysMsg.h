/*================================================================================

Module Name: SysMsg.h

General Description: ROS messager.

==================================================================================*/

#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>

namespace Msg
{

class MessagerObserver
{
public:
  // If using StartAsync/StopAsync way, OnMessage() may come from different threads
  virtual void OnMessage(const std::string &topic, const std::string &payload) = 0;
};

// Messager is thread safe
class Messager
{
public:
  static Messager& GetMessager();

  // Init() can be called at any time to get ros::ok() result but only
  // the first time to set the node name
  bool Init(int argc, char **argv, const std::string &node_name);

  // Must create all topics for publisher before calling Publish().
  // Normally CreatePublisherTopics() after Init()
  bool CreatePublisherTopics(const std::set<std::string> &topics);

  // Return false if the topic is not created successfully in CreateTopics()
  bool Publish(const std::string &topic, const std::string &payload);

  // Can subscribe on the same topic more than once, but still one copy
  // of the change on the topic
  bool Subscribe(const std::string &topic, MessagerObserver &observer);

  // Need RunOnce() repeatly to get all
  bool RunOnce(uint32_t hz_per_second = 20);

  // Will run forever
  bool Run();

  // Notify observers async in threads until StopAsync
  bool StartAsync(uint32_t thread_count = 0);
  bool StopAsync();

private:
  Messager() = default;
  ~Messager();

  // No copyable
  Messager(const Messager&) = delete;
  Messager& operator=(const Messager&) = delete;

  static void SignalHandler(int signum);

  class Listener
  {
  public:
    Listener(const std::string &topic,
             ros::NodeHandle &node_handle,
             std::recursive_mutex& mutex);
    void Callback(const std_msgs::String::ConstPtr &msg);
    std::string GetTopic() const;
    void AddObserver(MessagerObserver &observer);

  private:
    const std::string topic_;
    std::recursive_mutex& mutex_;
    std::shared_ptr<ros::Subscriber> subscriber_;
    std::set<MessagerObserver*> observers_;
  };

  std::shared_ptr<ros::NodeHandle> node_handle_;

  std::recursive_mutex mutex_;

  std::shared_ptr<ros::AsyncSpinner> spinner_;

  // Keep all publishers and subscribers with different topic until exit
  std::set<std::shared_ptr<ros::Publisher>> publishers_;
  std::set<std::shared_ptr<Listener>> listeners_;
};

} // namespace Msg
