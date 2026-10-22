/*================================================================================

Module Name: SysMsg.h

General Description: Dbus messager

==================================================================================*/

#pragma once

#include <sdbus-c++/sdbus-c++.h>

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>

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

  // Only the first call to init Dbus with the name
  bool Init(int argc, char **argv, const std::string &node_name);

  // Must create all topics for publisher before calling Publish().
  // Normally CreatePublisherTopics() after Init()
  bool CreatePublisherTopics(const std::set<std::string> &topics);

  // Return false if the topic is not created successfully in CreateTopics()
  bool Publish(const std::string &topic, const std::string &payload);

  // Can subscribe on the same topic more than once, but still one copy
  // of the change on the topic
  bool Subscribe(const std::string &topic, MessagerObserver &observer);

  // Notify observers async in threads until StopAsync
  bool StartAsync(uint32_t thread_count = 0);
  bool StopAsync();
  bool Stop();
private:
  Messager() = default;

  // No copyable
  Messager(const Messager&) = delete;
  Messager& operator=(const Messager&) = delete;

  static void SignalHandler(int signum);

  std::recursive_mutex mutex_;

  const std::string bus_name_ = "com.drone.messager";
  const std::string object_name_ = "/com/drone/messager";
  const std::string signal_name_ = "TopicUpdate";
  std::string node_name_;

  // For publisher
  std::unique_ptr<sdbus::IConnection> p_connection_;
  std::unique_ptr<sdbus::IObject> p_object_;
  std::set<std::string> p_topics_;

  // For subscriber
  std::unique_ptr<sdbus::IConnection> s_connection_;
  std::unique_ptr<sdbus::IProxy> s_proxy_;
  std::map<std::string, std::set<MessagerObserver*>> s_observers_;
  void OnTopicUpdate(sdbus::Signal& signal);
};

} // namespace Msg
