#include "SysMsg/SysMsg.h"

#include <signal.h>

namespace Msg
{

Messager& Messager::GetMessager()
{
  static Messager instance;
  return instance;
}

bool Messager::Init(int /*argc*/, char **/*argv*/, const std::string &node_name)
{
  if (node_name_.empty()) {
    node_name_ = node_name;
    signal(SIGINT, Messager::SignalHandler);
  }
  return true;
}

bool Messager::CreatePublisherTopics(const std::set<std::string> &topics)
{
  std::lock_guard<std::recursive_mutex> guard(mutex_);

  if (!p_connection_) {
//  p_connection_ = sdbus::createSessionBusConnection(bus_name_);
    p_connection_ = sdbus::createSystemBusConnection(bus_name_);
    p_object_ = sdbus::createObject(*p_connection_, object_name_);
    p_object_->registerSignal(signal_name_).onInterface(bus_name_).withParameters<std::string, std::string>();
    p_object_->finishRegistration();
  }
  for (const auto& t : topics) {
    p_topics_.insert(t);
  }
  return true;
}

bool Messager::Publish(const std::string &topic, const std::string &payload)
{
  std::lock_guard<std::recursive_mutex> guard(mutex_);

  if (p_topics_.find(topic) == p_topics_.end()) {
    return false;
  }

  p_object_->emitSignal(signal_name_).onInterface(bus_name_).withArguments(topic, payload);
  return true;
}

bool Messager::Subscribe(const std::string &topic, MessagerObserver &observer)
{
  std::lock_guard<std::recursive_mutex> guard(mutex_);

  auto it = s_observers_.find(topic);
  if (it == s_observers_.end()) {
    std::set<MessagerObserver*> o;
    o.insert(&observer);
    s_observers_.emplace(topic, std::move(o));
  }
  else {
    it->second.insert(&observer);
  }
  return true;
}

// TODO: thread_count is not implemented
bool Messager::StartAsync(uint32_t /*thread_count*/)
{
  std::lock_guard<std::recursive_mutex> guard(mutex_);

  if (!s_connection_) {
//    s_connection_ = sdbus::createSessionBusConnection();
    s_connection_ = sdbus::createSystemBusConnection();
    s_proxy_ = sdbus::createProxy(*s_connection_, bus_name_, object_name_);
    s_proxy_->registerSignalHandler(
      bus_name_, signal_name_,
      std::bind(&Messager::OnTopicUpdate, this, std::placeholders::_1));
    s_proxy_->finishRegistration();
    s_connection_->enterEventLoopAsync();
  }
  return true;
}

bool Messager::StopAsync()
{
  std::lock_guard<std::recursive_mutex> guard(mutex_);

  if (s_connection_) {
    s_connection_->leaveEventLoop();
  }
  return true;
}

bool Messager::Stop()
{
  return StopAsync();
}

void Messager::SignalHandler(int signum)
{
  GetMessager().StopAsync();
  exit(0);
}

void Messager::OnTopicUpdate(sdbus::Signal& signal)
{
  std::string topic, payload;
  signal >> topic >> payload;

  // Copy to local for thread safe
  std::set<MessagerObserver*> observers_copy;
  {
    // Can't put to the begining of this function otherwise one long OnMessage()
    // may block other OnMessage()
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    auto it = s_observers_.find(topic);
    if (it != s_observers_.end()) {
      observers_copy = it->second;
    }
  }

  for (auto &each : observers_copy) {
    each->OnMessage(topic, payload);
  }
}

} // namespace Msg
