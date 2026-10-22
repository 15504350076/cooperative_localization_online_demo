#include "SysMsg/SysMsg.h"

#include <iostream>
#include <signal.h>

namespace Msg
{

const uint32_t MESSAGE_QUEUE_SIZE = 1000;
const double WAITING_TOPIC_READY_TIME_S = 3.0;

Messager& Messager::GetMessager()
{
  static Messager instance;
  return instance;
}

Messager::~Messager()
{
  publishers_.clear();
  listeners_.clear();
  spinner_.reset();
  node_handle_.reset();
}

bool Messager::Init(int argc, char **argv, const std::string &node_name)
{
  // Init only oonce
  if (!node_handle_) {
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
    node_handle_ = std::make_shared<ros::NodeHandle>();
    signal(SIGINT, Messager::SignalHandler);
  }
  return ros::ok();
}

bool Messager::CreatePublisherTopics(const std::set<std::string> &topics)
{
  if (!ros::ok()) {
    return false;
  }

  std::lock_guard<std::recursive_mutex> guard(mutex_);

  for (auto& topic : topics) {
    bool existed = false;
    for (auto &publisher : publishers_) {
      if (publisher->getTopic() == "/" + topic) {
        existed = true;
        break;
      }
    }

    if (!existed) {
      auto publisher = std::make_shared<ros::Publisher>(
        node_handle_->advertise<std_msgs::String>(topic, MESSAGE_QUEUE_SIZE));
      publishers_.emplace(publisher);
    }
  }

  // Need wait a while for publisher ready. Otherwise the message may be lost
  // if publishing quickly after creating
  ros::Duration(WAITING_TOPIC_READY_TIME_S).sleep();
  return true;
}

bool Messager::Publish(const std::string &topic, const std::string &payload)
{
  if (!ros::ok()) {
    return false;
  }

  std::lock_guard<std::recursive_mutex> guard(mutex_);

  for (auto &each : publishers_) {
    if (each->getTopic() == "/" + topic) {
      std_msgs::String msg;
      msg.data = payload;
      each->publish(msg);
      return true;
    }
  }

  return false;
}

bool Messager::Subscribe(const std::string &topic, MessagerObserver &observer)
{
  if (!ros::ok()) {
    return false;
  }

  std::lock_guard<std::recursive_mutex> guard(mutex_);

  for (auto &each : listeners_) {
    if (each->GetTopic() == "/" + topic) {
      // Has subscribed the topic already
      each->AddObserver(observer);
      return true;
    }
  }

  auto listener = std::make_shared<Listener>(topic, *node_handle_, mutex_);
  listener->AddObserver(observer);
  listeners_.emplace(listener);
  return true;
}

bool Messager::RunOnce(uint32_t hz_per_second)
{
  if (!ros::ok()) {
    return false;
  }

  ros::Rate loop_rate(hz_per_second);
  ros::spinOnce();
  loop_rate.sleep();
  return true;
}

bool Messager::Run()
{
  if (!ros::ok()) {
    return false;
  }

  ros::spin();
  return true; // Fine, no chance to return
}

bool Messager::StartAsync(uint32_t thread_count)
{
  if (!ros::ok()) {
    return false;
   }

  std::lock_guard<std::recursive_mutex> guard(mutex_);
  if (spinner_) {
    spinner_->stop();
  }
  spinner_ = std::make_shared<ros::AsyncSpinner>(thread_count);
  spinner_->start();
  return true;
}

bool Messager::StopAsync()
{
  if (!ros::ok()) {
    return false;
  }

  std::lock_guard<std::recursive_mutex> guard(mutex_);
  if (spinner_) {
    spinner_->stop();
    spinner_.reset();
  }
  return true;
}

void Messager::SignalHandler(int /*signum*/)
{
  GetMessager().StopAsync();
  ros::shutdown();
  exit(0);
}

Messager::Listener::Listener(const std::string &topic,
                             ros::NodeHandle &node_handle,
                             std::recursive_mutex& mutex)
  : topic_(topic),
    mutex_(mutex)
{
  subscriber_ = std::make_shared<ros::Subscriber>(
    node_handle.subscribe(topic, MESSAGE_QUEUE_SIZE,
                          &Listener::Callback, this));
}

std::string Messager::Listener::GetTopic() const
{
  return subscriber_->getTopic();
}

// May from different threads if using StartAsync/StopAsync way
void Messager::Listener::Callback(const std_msgs::String::ConstPtr &msg)
{
  // Copy to local for thread safe. OnMessage() may be called
  // fromn different threads but not need protect it, otherwise
  // OnMessage() will be called one by one. One long OnMessage()
  // may block other OnMessage(). This is not the expected behavior.
  // Caller selects StartAsync/StopAsync way, then the expected behavior
  // is parallel OnMessage() from different threads, otherwise can use
  // Run() or RunOnce()
  std::set<MessagerObserver*> observers_copy;
  {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    observers_copy = observers_;
  }

  for (auto &each : observers_copy) {
    each->OnMessage(topic_, msg->data);
  }
}

void Messager::Listener::AddObserver(MessagerObserver &observer)
{
  // Now AddObserver is called in Subscribe() with mutex_ protection
  observers_.emplace(&observer);
}

} // namespace Msg
