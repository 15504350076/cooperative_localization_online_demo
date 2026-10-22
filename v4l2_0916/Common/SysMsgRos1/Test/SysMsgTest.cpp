#include "SysMsg/SysMsg.h"

#include <iostream>
#include <thread>

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
    std::cout << "Observer A: " << topic << " : " << payload << std::endl;
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
    std::cout << "Observer AB: " << topic << " : " << payload << std::endl;
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
    std::cout << "Observer All: " << topic << " : " << payload << std::endl;
  }
};


void RunOnce_Thread(bool &stop)
{
  do {
    Msg::Messager::GetMessager().RunOnce();
  } while (!stop);
}

int main(int argc, char* argv[])
{
  const std::string json1 =
    "{"
    "  'json1_sites': ["
    "  { 'name':'菜鸟教程' , 'url':'www.runoob.com' },"
    "  { 'name':'google' , url':'www.google.com' },"
    "  { 'name':'微博' , 'url':'www.weibo.com' }"
    "  ]"
    "}";

  const std::string json2 =
    "{"
    "  'json2_employees:["
    " {'firstName':'John', 'lastName':'Doe'},"
    " {'firstName':'Anna', 'lastName':'Smith'},"
    " {'firstName':'Peter', 'lastName':'Jones'}"
    " ]"
    "}";

  auto& messager = Msg::Messager::GetMessager();
  messager.Init(argc, argv, "drone");
  messager.CreatePublisherTopics({"topic_a", "topic_b", "topic_c", "topic_d"});
  std::cout<< "Ready" << std::endl;

  ObserverA observer_a;
  ObserverAB observer_ab;

  ObserverAll observer_all;
  // observer_all has subscribed on "topic_d". It is fine to subscribe again
  // but still get one copy instead of two. Check the log to confirm that only
  // one OnMessage() for "topic_d"
  messager.Subscribe("topic_d", observer_all);

  messager.Publish("topic_a", json1);
  messager.Publish("topic_b", json2);
  messager.Publish("topic_a", json2);
  messager.Publish("topic_b", json1);
  messager.Publish("topic_a", json1);
  messager.Publish("topic_d", json2);

  // Way 1: block forever
  //messager.Run();

  // Way 2: notify OnMessage() in one thread
  bool stop = false;
  std::thread thread(RunOnce_Thread, std::ref(stop));
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  stop = true;
  thread.join();

  // Way 3: no block and notify OnMessage() in different threads
  /*
  messager.StartAsync();
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  messager.StopAsync();
  */

  return EXIT_SUCCESS;
}
