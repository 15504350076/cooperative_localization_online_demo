#include "SysMsg/SysMsg.h"

#include <iostream>
#include <thread>

std::recursive_mutex mutex_;

class ObserverA : public Msg::MessagerObserver
{
public:
  ObserverA()
  {
    // Only care about topic_a
    Msg::Messager::GetMessager().Subscribe("topic_a", *this);
  }

  void OnMessage(const std::string &topic, const std::string &payload) override
  {
    // Simulate a block in topic_a
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    std::cout << "(thread id: " << std::this_thread::get_id() << ") Observer A: " << topic << " : " << payload << std::endl;
  }
};

class ObserverAB : public Msg::MessagerObserver
{
public:
  ObserverAB()
  {
    // Care about topic_a and topic_b
    Msg::Messager::GetMessager().Subscribe("topic_a", *this);
    Msg::Messager::GetMessager().Subscribe("topic_b", *this);
  }

  void OnMessage(const std::string &topic, const std::string &payload) override
  {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    std::cout << "(thread id: " << std::this_thread::get_id() << ") Observer AB: " << topic << " : " << payload << std::endl;
  }
};

class ObserverAll : public Msg::MessagerObserver
{
public:
  ObserverAll()
  {
    // Care about all topics
    Msg::Messager::GetMessager().Subscribe("topic_a", *this);
    Msg::Messager::GetMessager().Subscribe("topic_b", *this);
    Msg::Messager::GetMessager().Subscribe("topic_c", *this);
    Msg::Messager::GetMessager().Subscribe("topic_d", *this);
  }

  void OnMessage(const std::string &topic, const std::string &payload) override
  {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    std::cout << "(thread id: " << std::this_thread::get_id() << ") Observer All: " << topic << " : " << payload << std::endl;
  }
};

int main(int argc, char* argv[])
{
  auto& messager = Msg::Messager::GetMessager();
  messager.Init(argc, argv, "drone");
  messager.CreatePublisherTopics({"topic_a", "topic_b", "topic_c", "topic_d"});
  messager.StartAsync();
  std::cout<< "Ready" << std::endl;

  ObserverA observer_a;
  ObserverAB observer_ab;

  ObserverAll observer_all;
  // observer_all has subscribed on "topic_d". It is fine to subscribe again
  // but still get one copy instead of two. Check the log to confirm that only
  // one OnMessage() for "topic_d"
  messager.Subscribe("topic_d", observer_all);

  // Check the log to confirm:
  // topic_a will be blocked in ObserverA, but other topics will not,
  // so topic_a is published at first but log in the end.
  messager.Publish("topic_a", "topic_a 1");
  messager.Publish("topic_b", "topic_b 1");
  messager.Publish("topic_c", "topic_c 1");
  messager.Publish("topic_d", "topic_d 1");
  messager.Publish("topic_a", "topic_a 2");
  messager.Publish("topic_b", "topic_b 2");
  messager.Publish("topic_c", "topic_c 2");

  std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  std::cout << "You can wait to exit or Press CTRL+c to exit" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(10000));
  messager.Stop();

  return EXIT_SUCCESS;
}
